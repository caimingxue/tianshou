from tianshou.data import ReplayBuffer, Batch
import numpy as np
buf = ReplayBuffer(size=20)
for i in range(15):
    buf.add(Batch(obs = i * 3, act = i, rew = i, done = 0, obs_next = i+1, info={}))
print(buf)
batch_data, indices = buf.sample(batch_size=18)
print(batch_data)
print(indices)
print(batch_data.obs == buf[indices].obs)