"""Restore scalar packet identities lost by NPZ serialization, no payload edits."""
def restore_packet_scalars(packet):
 out=dict(packet);out['frame']=int(packet['frame']);out['time_s']=float(packet['time_s']);return out
