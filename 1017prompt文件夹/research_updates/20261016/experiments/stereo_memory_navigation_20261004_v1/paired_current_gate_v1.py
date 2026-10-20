"""Implement frozen nonempty-precision and original coverage/retention gates exactly."""
def check_gates(rows,originalvalid,retainedvalid,originalbins,retainedbins,expected_epochs):
 nonempty=[r for r in rows if r['eligible']>0]
 return dict(all_epochs_preserved=len(rows)==expected_epochs,each_nonempty_epoch_ge95=bool(nonempty) and all(r['precision'] is not None and r['precision']>=.95 for r in nonempty),original_GTvalid_retention_ge95=originalvalid>0 and retainedvalid/originalvalid>=.95,original_GTvalid_XY10cm_bin_retention_ge95=originalbins>0 and retainedbins/originalbins>=.95,two_eligible100hazard_epochs=sum(r['eligible_hazard']>=100 for r in rows)>=2)
