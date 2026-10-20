"""Small Python symbol-table audit for missing module globals in runners."""
import builtins,symtable

def unbound_global_names(source):
 root=symtable.symtable(source,'<runner>','exec');bound=set(dir(builtins))|{s.get_name() for s in root.get_symbols() if s.is_assigned() or s.is_imported() or s.is_namespace()}|{'__file__','__name__'};missing=set()
 def visit(table):
  for s in table.get_symbols():
   if s.is_global() and s.is_referenced() and s.get_name() not in bound:missing.add(s.get_name())
  for c in table.get_children():visit(c)
 visit(root);return sorted(missing)
