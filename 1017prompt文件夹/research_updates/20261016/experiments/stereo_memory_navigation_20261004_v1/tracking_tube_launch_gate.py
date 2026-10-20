"""Review closure belongs to this frozen source set, not regenerated evidence."""
def verify_frozen_review(protocol,suite,review,sha):
 paths=protocol['review_source_paths'];closed=review['closed_source_sha256']
 if not paths or len(paths)!=len(set(paths)) or set(closed)!=set(paths):raise ValueError('complete unique review source binding required')
 if review['unresolved_critical'] or review['unresolved_important']:raise ValueError('unresolved review')
 for p in paths:
  if not closed[p]==protocol['source_sha256'].get(p)==suite['source_sha256'].get(p)==sha(p):raise ValueError('stale review/source binding')
