
from scipy.sparse import lil_matrix
import numpy as np

num_observations = 10
num_tags = 5
obs_tag_idxs = np.random.randint(0, num_tags, num_observations)

sparsity_step1 = lil_matrix((num_observations * 3, num_tags * 3), dtype=int)
for i in range(num_observations):
    sparsity_step1[i*3:(i+1)*3, obs_tag_idxs[i]*3:(obs_tag_idxs[i]+1)*3] = 1

print("Success")
