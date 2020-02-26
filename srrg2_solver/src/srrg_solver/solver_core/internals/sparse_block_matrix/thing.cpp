    // compute elimination tree
    std::cerr << "elimination tree";
    std::vector<int> parents(blockCols());
    std::vector<std::list<int> > children(blockCols());
    for (int col_idx=0; col_idx<blockCols(); ++col_idx){
      IntBlockMap& ch_col = _cols[col_idx];
      if (ch_col.empty()){
	return false;
      }
      auto row_it=ch_col.begin();
      row_it++;
      if (row_it==ch_col.end()) {
	parents[col_idx] = -1;
      } else {
	parents[col_idx] = row_it->first;
	children[row_it->first].push_back(col_idx);
      }
    }
    std::cerr << "copying structure" << std::endl;
    // reflect matrix structure
    std::vector<std::set<int> > row_structures(blockRows());
    for (int col_idx=0; col_idx<blockCols(); ++col_idx){
      IntBlockMap& ch_col = _cols[col_idx];
      std::set<int>& row_structure=row_structures[col_idx];
      for (auto ch_it=ch_col.begin(); ch_it!=ch_col.end(); ++ch_it) {
	row_structure.insert(ch_it->first);
      }
    }

    std::cerr << "computing structure" << std::endl;
    for (int col_idx=0; col_idx<blockCols(); ++col_idx){
      std::set<int>& row_structure=row_structures[col_idx];
      std::cerr << "col_idx:" << col_idx << std::endl;
     for (int child_idx: children[col_idx]) {
	std::cerr << "child_idx:" << child_idx << std::endl;
	const std::set<int>& child_structure=row_structures[child_idx];
	auto child_start_it=child_structure.upper_bound(col_idx-1);
	row_structure.insert(child_start_it,
			     child_structure.end());
      }
    }
    std::cerr << "done" << endl;
 
    
