import os
import gym
import torch
import pprint
import argparse
import numpy as np
from torch.utils.tensorboard import SummaryWriter

from tianshou.policy import TD3Policy
from tianshou.utils import BasicLogger
from tianshou.env import DummyVectorEnv
from tianshou.utils.net.common import Net
from tianshou.trainer import offpolicy_trainer
from tianshou.exploration import GaussianNoise
from tianshou.data import Collector, VectorReplayBuffer
from tianshou.utils.net.continuous import Actor, Critic

from pybullet_envs.bullet.kuka_diverse_object_gym_env import KukaDiverseObjectEnv


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--task', type=str, default='LunarLanderContinuous-v2')
    parser.add_argument('--seed', type=int, default=1)
    parser.add_argument('--buffer-size', type=int, default=20000)
    parser.add_argument('--actor-lr', type=float, default=1e-4)
    parser.add_argument('--critic-lr', type=float, default=1e-3)
    parser.add_argument('--gamma', type=float, default=0.99)
    parser.add_argument('--tau', type=float, default=0.005)
    parser.add_argument('--exploration-noise', type=float, default=0.1)
    parser.add_argument('--policy-noise', type=float, default=0.2)
    parser.add_argument('--noise-clip', type=float, default=0.5)
    parser.add_argument('--update-actor-freq', type=int, default=2)
    parser.add_argument('--epoch', type=int, default=5)
    parser.add_argument('--step-per-epoch', type=int, default=150000)
    parser.add_argument('--step-per-collect', type=int, default=8)
    parser.add_argument('--update-per-step', type=float, default=0.125)
    parser.add_argument('--batch-size', type=int, default=128)
    parser.add_argument('--hidden-sizes', type=int, nargs='*', default=[128, 128])
    parser.add_argument('--training-num', type=int, default=20)
    parser.add_argument('--test-num', type=int, default=10)
    parser.add_argument('--logdir', type=str, default='log')
    parser.add_argument('--render', type=float, default=0.001)
    parser.add_argument('--rew-norm', action="store_true", default=False)
    parser.add_argument('--n-step', type=int, default=3)
    parser.add_argument(
        '--device', type=str,
        default='cuda' if torch.cuda.is_available() else 'cpu')
    args = parser.parse_known_args()[0]
    return args


def test_td3(args=get_args()):
    torch.set_num_threads(1)  # we just need only one thread for NN
    env = gym.make(args.task)
    if args.task == 'Pendulum-v0':
        env.spec.reward_threshold = -250
    if args.task == 'LunarLanderContinuous-v2':
        env.spec.reward_threshold = -10
    args.state_shape = env.observation_space.shape or env.observation_space.n
    args.action_shape = env.action_space.shape or env.action_space.n
    args.max_action = env.action_space.high[0]
    # you can also use tianshou.env.SubprocVectorEnv
    # train_envs = gym.make(args.task)
    train_envs = DummyVectorEnv(
        [lambda: gym.make(args.task) for _ in range(args.training_num)])
    # test_envs = gym.make(args.task)
    test_envs = DummyVectorEnv(
        [lambda: gym.make(args.task) for _ in range(args.test_num)])
    # seed
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    train_envs.seed(args.seed)
    test_envs.seed(args.seed)
    # model
    net = Net(args.state_shape, hidden_sizes=args.hidden_sizes,
              device=args.device)
    actor = Actor(net, args.action_shape, max_action=args.max_action,
                  device=args.device).to(args.device)
    actor_optim = torch.optim.Adam(actor.parameters(), lr=args.actor_lr)
    net_c1 = Net(args.state_shape, args.action_shape,
                 hidden_sizes=args.hidden_sizes,
                 concat=True, device=args.device)
    critic1 = Critic(net_c1, device=args.device).to(args.device)
    critic1_optim = torch.optim.Adam(critic1.parameters(), lr=args.critic_lr)
    net_c2 = Net(args.state_shape, args.action_shape,
                 hidden_sizes=args.hidden_sizes,
                 concat=True, device=args.device)
    critic2 = Critic(net_c2, device=args.device).to(args.device)
    critic2_optim = torch.optim.Adam(critic2.parameters(), lr=args.critic_lr)
    policy = TD3Policy(
        actor, actor_optim, critic1, critic1_optim, critic2, critic2_optim,
        tau=args.tau, gamma=args.gamma,
        exploration_noise=GaussianNoise(sigma=args.exploration_noise),
        policy_noise=args.policy_noise,
        update_actor_freq=args.update_actor_freq,
        noise_clip=args.noise_clip,
        reward_normalization=args.rew_norm,
        estimation_step=args.n_step,
        action_space=env.action_space)
    # collector
    train_collector = Collector(
        policy, train_envs,
        VectorReplayBuffer(args.buffer_size, len(train_envs)),
        exploration_noise=True)
    test_collector = Collector(policy, test_envs)
    # train_collector.collect(n_step=args.buffer_size)
    # log
    log_path = os.path.join(args.logdir, args.task, 'td3')
    writer = SummaryWriter(log_path)
    logger = BasicLogger(writer)

    def save_fn(policy):
        torch.save(policy.state_dict(), os.path.join(log_path, 'policy.pth'))

    def stop_fn(mean_rewards):
        return mean_rewards >= env.spec.reward_threshold

    # trainer
    result = offpolicy_trainer(
        policy, train_collector, test_collector, args.epoch,
        args.step_per_epoch, args.step_per_collect, args.test_num, args.batch_size,
        update_per_step=args.update_per_step, stop_fn=stop_fn,
        save_fn=save_fn, logger=logger)
    assert stop_fn(result['best_reward'])

    if __name__ == '__main__':
        pprint.pprint(result)
        # Let's watch its performance!
        env = gym.make(args.task)
        policy.eval()
        collector = Collector(policy, env)
        result = collector.collect(n_episode=5, render=args.render)
        rews, lens = result["rews"], result["lens"]
        print(f"Final reward: {rews.mean()}, length: {lens.mean()}")


if __name__ == '__main__':
    test_td3()
