"""Complete-sequence partitions and equal-update data-expansion sampling."""
import numpy as np

def batch_schedule(indices, epochs, batch_size, seed):
    if epochs<1 or batch_size<1 or not indices or len(set(indices))!=len(indices) or len(indices)%batch_size:
        raise ValueError('Require unique indices, positive epochs and complete batches')
    rng=np.random.RandomState(seed)
    for _ in range(epochs):
        order=rng.permutation(indices)
        for start in range(0,len(order),batch_size):
            yield order[start:start+batch_size].tolist()

def expanded_partition(metadata, partitions, extra_metadata):
    if len(partitions)!=3 or sorted(i for group in partitions for i in group)!=list(range(len(metadata))):
        raise ValueError('Partitions must cover the original data exactly once')
    groups=[{metadata[i]['episode'] for i in ids} for ids in partitions]
    new={m['episode'] for m in extra_metadata}
    if any(groups[i]&groups[j] for i in range(3) for j in range(i)) or new & set.union(*groups):
        raise ValueError('Require disjoint whole sequences')
    return [list(partitions[0])+list(range(len(metadata),len(metadata)+len(extra_metadata))),list(partitions[1]),list(partitions[2])]
