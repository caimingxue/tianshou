from torch.utils.tensorboard import SummaryWriter
import numpy as np
import datetime
import sys,os
curr_path = os.path.dirname(__file__)

curr_time = datetime.datetime.now().strftime("%Y%m%d-%H%M%S") # obtain current time

writer = SummaryWriter("tensorboard_log/"+curr_time)

# for n_iter in range(100):
#     writer.add_scalar('Loss/train', np.random.random(), n_iter)
#     writer.add_scalar('Loss/test', np.random.random(), n_iter)
#     writer.add_scalar('Accuracy/train', np.random.random(), n_iter)
#     writer.add_scalar('Accuracy/test', np.random.random(), n_iter)
# writer.close()

r = 5
for i in range(100):
    writer.add_scalars('run_14h', {'xsinx':i*np.sin(i/r),
                                    'xcosx':i*np.cos(i/r),
                                    'tanx': np.tan(i/r)}, i)

# for i in range(10):
#     x = np.random.random(1000)
#     writer.add_histogram('distribution centers', x + i, i)
writer.close()