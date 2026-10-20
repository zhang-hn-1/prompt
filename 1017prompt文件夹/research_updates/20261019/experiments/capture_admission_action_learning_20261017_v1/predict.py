"""All scores sealed before postfit teacher scoring. Features only, no label reads."""
from common import *
from policy_model import AdmissionActionModel
def main():
    cfg=load(D/'protocol.json');ws=load(D/'weight_seal.json');assert ws['fit_converged'] and sha(D/'weights.json')==ws['weight_sha256']
    model=AdmissionActionModel(load(D/'weights.json'));fs=load(cfg['feature_seal']);out=D/'predictions';out.mkdir();rows=[];total=0
    for f in fs['rows']:
        assert sha(f['feature_path'])==f['feature_sha256']
        with np.load(f['feature_path']) as a:
            scores=model.score(a['features']);assert np.isfinite(scores).all() and ((scores>=0)&(scores<=1)).all()
            path=out/(f['sequence']+'_'+Path(f['feature_path']).name)
            np.savez_compressed(path,scores=scores,cells=a['cells'],origin=a['origin'],time_s=a['time_s'])
            total+=len(scores);rows.append(dict(sequence=f['sequence'],frame=f['frame'],time_s=f['time_s'],path=str(path),sha256=sha(path),feature_path=f['feature_path'],feature_sha256=f['feature_sha256'],cells=len(scores),predicted_WITHHOLD_NEW=int((scores>=.95).sum())))
    assert len(rows)==470 and total==40052
    # Numeric API guard: invalid model-input rows produce unavailable score -> HOLD.
    assert np.isnan(model.score(np.full((1,24),np.nan))[0])
    put(D/'prediction_seal.json',dict(complete=True,utc=now(),rows=rows,cell_scores=total,weight_seal_sha256=sha(D/'weight_seal.json'),weight_sha256=sha(D/'weights.json'),
        teacher_labels_read_in_prediction=False,source_calls=0,heldout_input_read=False,invalid_features_score_NaN=True))
    print('ALL470_ACTION_SCORES_SEALED',total,flush=True)
if __name__=='__main__':main()
