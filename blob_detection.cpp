#include "blob_detection.h"

double COLOR_THRESHOLD = 20;
double TEMPLATE_HUE    = 180;

bool px_match(uint32_t base, uint32_t test) {
	return color_fit(COLOR_THRESHOLD,
			TEMPLATE_HUE, test);
}

bool color_fit(double color_threshold,
					 double template_hue,
					 uint32_t px) {
	return hue_dist(template_hue, px) < color_threshold;
}

void getNLabels(int n_labels[], int labels[], int neighbors[], int len_neighbors) {
	for(int i = 0; i < len_neighbors; ++i) {
		n_labels[i] = labels[neighbors[i]];
	}
}

int minLabel(int n_labels[], int len_labels) {
	int minLabel = n_labels[0];
	for(int i = 1; i < len_labels; ++i) {
		if(n_labels[i] < minLabel) minLabel = n_labels[i];
	}
	return minLabel;
}

void unionLabels(std::vector<Set *> links, std::vector<int> n_labels) {
	//Pass over twice
	//1st time gets all set to first neighbor
	//2nd updates rest
	if(!n_labels.empty()) {
		if(n_labels[0] < (int)links.size()) {
			for(int i = 0; i < 2; ++i) {
				for(size_t j = 1; j < n_labels.size(); ++j) {
					if(n_labels[j] < (int)links.size()) {
						links[n_labels[0]]->unionS(links[n_labels[j]]);
					}
				}
			}
		}
	}
}

std::vector<std::vector<blob_t>> blob_detection(Image<uint32_t> &im, std::vector<blob_type_t> &blob_types) {
	//Set globals
	//Nothing should use this
	std::vector<std::vector<Set *>> links(blob_types.size(), std::vector<Set *>());

	for (unsigned int i = 0; i < blob_types.size(); i++) {
		links[i].push_back(new Set(0));
	}

	//Hold data for blobs
	std::vector<std::vector<blob_t>> all_blobs(blob_types.size(), std::vector<blob_t>());
	//each px has a label, 0 is default
	std::vector<int> single_labels(im.size(),0);
	std::vector<std::vector<int>> labels(blob_types.size(), single_labels);
	//These are the final labels that are detected as objects,
	//used for modifying the image
	std::vector<std::vector<int>>  final_labels(blob_types.size(), std::vector<int>());
	//Incremented when new labels found
	std::vector<int> label_num(blob_types.size(), 1);
	static const std::vector<bool> search_neighbors = 
	{1,1,1,
	 1,  0,
	 0,0,0};
	//1st pass
	for(size_t i = 0; i < im.size(); ++i) {
		uint32_t px = im.get(i);
		for (unsigned bc = 0; bc < blob_types.size(); bc++) {
			blob_type &bt = blob_types[bc];
			if(color_fit(bt.color_threshold, bt.template_hue, px)
					&& im.isValid(i)) {
				COLOR_THRESHOLD = bt.color_threshold;
				TEMPLATE_HUE    = bt.template_hue;
				std::vector<int> neighbors =
					im.getNeighborIds(im.getX(i),im.getY(i),
							search_neighbors, px_match);
				if(!neighbors.empty()) {
					std::vector<int> n_labels;
					assert(neighbors.size() <= 4);
					for(size_t j = 0; j < neighbors.size(); ++j) {
						assert(im.getY(neighbors[j]) <= im.getY(i));
						n_labels.push_back(labels[bc][neighbors[j]]);
					}
					labels[bc][i] = *std::min_element(n_labels.begin(), n_labels.end());
					unionLabels(links[bc], n_labels);
				} else {
					labels[bc][i] = label_num[bc];
					links[bc].push_back(new Set(label_num[bc]));
					label_num[bc] ++;
				}
			} else {
				labels[bc][i] = 0;
			}
		}
	}

	std::vector<std::vector<blob_t>> ret_blobs;
	for (unsigned int bc = 0; bc < blob_types.size(); bc++) {
		blob_type &bt = blob_types[bc];
		//Init balls
		std::vector<blob_t> &blobs = all_blobs[bc];
		std::vector<int> &cur_labels = labels[bc];
		for(int i = 0; i <= label_num[bc]; ++i) {
			blob_t temp;
			blobs.push_back(temp);
			blobs[i].x = 0;
			blobs[i].y = 0;

			blobs[i].t.x = 0;
			blobs[i].t.y = 0;

			blobs[i].b.x = 0;
			blobs[i].b.y = im.h();

			blobs[i].l.x = im.w();
			blobs[i].l.y = 0;

			blobs[i].r.x = 0;
			blobs[i].r.y = 0;

			blobs[i].valid  = 1;
			blobs[i].num_px = 0;
		}
		//2nd pass
		for(size_t i = 0; i < im.size(); ++i) {
			uint32_t px = im.get(i);
			int x = im.getX(i);
			int y = im.getY(i);
			if(color_fit(bt.color_threshold, bt.template_hue, px)
					&& im.isValid(i)
					&& (int)links[bc].size() > cur_labels[i]
					&& cur_labels[i] > 0){
				cur_labels[i] = links[bc][cur_labels[i]]->findS()->get();
				//Add ball data
				blobs[cur_labels[i]].x += x;
				blobs[cur_labels[i]].y += y;
				blobs[cur_labels[i]].num_px ++;
				//Update t,b,l,r
				if(x > blobs[cur_labels[i]].r.x) {
					blobs[cur_labels[i]].r.x = x;
					blobs[cur_labels[i]].r.y = y;
					blobs[cur_labels[i]].valid = 1;
					//If == then not our diamond
				} else if(x == blobs[cur_labels[i]].r.x){
					blobs[cur_labels[i]].valid = 0;
				}
				if(x < blobs[cur_labels[i]].l.x) {
					blobs[cur_labels[i]].l.x = x;
					blobs[cur_labels[i]].l.y = y;
					blobs[cur_labels[i]].valid = 1;
				} else if(x == blobs[cur_labels[i]].l.x){
					blobs[cur_labels[i]].valid = 0;
				}
				if(y > blobs[cur_labels[i]].t.y) {
					blobs[cur_labels[i]].t.x = x;
					blobs[cur_labels[i]].t.y = y;
					blobs[cur_labels[i]].valid = 1;
				} else if(y == blobs[cur_labels[i]].t.y){
					blobs[cur_labels[i]].valid = 0;
				}
				if(y < blobs[cur_labels[i]].b.y) {
					blobs[cur_labels[i]].b.x = x;
					blobs[cur_labels[i]].b.y = y;
					blobs[cur_labels[i]].valid = 1;
				} else if(y == blobs[cur_labels[i]].b.y){
					blobs[cur_labels[i]].valid = 0;
				}
			}
		}

		//Filter out
		int most_px    = 0;
		int final_num_blobs = 0;
		std::vector<blob_t> temp_blobs;
		for(int i = 1; i < label_num[bc]; ++i) {
			//Assert right num_pxs
			int right_num_pxs = (blobs[i].num_px >= bt.min_pxs &&
								 blobs[i].num_px <= MAX_PXS);

			int width = pixel_width(blobs[i].l,blobs[i].r);
			int height = pixel_height(blobs[i].t,blobs[i].b);

			double density = (blobs[i].num_px + 0.0) / (width*height+0.0);
			int dense_enough  = density > 0.25;

			if(right_num_pxs && dense_enough) {
				if(blobs[i].num_px > most_px) {
					most_px = blobs[i].num_px;
					//largest_idx = final_num_blobs;
				}

				temp_blobs.push_back(blobs[i]);
				//Get coordinates, not sum
				temp_blobs[final_num_blobs].x
					= (temp_blobs[final_num_blobs].x+0.0)/
					temp_blobs[final_num_blobs].num_px;
				temp_blobs[final_num_blobs].y =
					(temp_blobs[final_num_blobs].y+0.0)/
					temp_blobs[final_num_blobs].num_px;
				final_labels[bc].push_back(i);
				final_num_blobs ++;
			}
		}
		//Mark objects
		for(size_t i = 0; i < im.size(); ++i) {
			int t_label = cur_labels[i];
			//bool set = false;
			if(color_fit(bt.color_threshold,bt.template_hue,im.get(i))) {
				for(int z = 0; z < final_num_blobs; ++z) {
					if(t_label == final_labels[bc][z]) {
						//set = true;
						im.set(i,bt.show_px);
						break;
					}
				}
			}
		}
		//Clean up
		for(size_t i = 0; i < links[bc].size(); ++i) {
			delete links[bc][i];
		}
		ret_blobs.push_back(temp_blobs);
	}

	

	return ret_blobs;
}

