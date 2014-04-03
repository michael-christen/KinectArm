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

std::vector<blob_t> blob_detection(Image<uint32_t> &im, 
		double template_hue, uint32_t show_px,
		double color_threshold, int min_pxs) {
	//Set globals
	COLOR_THRESHOLD = color_threshold;
	TEMPLATE_HUE    = template_hue;
	//list of links b/t labels
	std::vector<Set *> links;
	//Nothing should use this
	links.push_back(new Set(0));
	//Hold data for blobs
	std::vector<blob_t> blobs;
	std::vector<blob_t> temp_blobs;
	//each px has a label, 0 is default
	std::vector<int> labels(im.size(),0);
	//These are the final labels that are detected as objects,
	//used for modifying the image
	std::vector<int> final_labels;
	//Incremented when new labels found
	int label_num = 1;
	static const std::vector<bool> search_neighbors = 
	{1,1,1,
	 1,  0,
	 0,0,0};
	int total_pxs = 0;
	//1st pass
	for(size_t i = 0; i < im.size(); ++i) {
		uint32_t px = im.get(i);
		if(color_fit(color_threshold, template_hue, px)
				&& im.isValid(i)
				) {
			total_pxs ++;
			std::vector<int> neighbors =
				im.getNeighborIds(im.getX(i),im.getY(i),
						search_neighbors, px_match);
			if(!neighbors.empty()) {
				std::vector<int> n_labels;
				assert(neighbors.size() <= 4);
				for(size_t j = 0; j < neighbors.size(); ++j) {
					assert(im.getY(neighbors[j]) <= im.getY(i));
					n_labels.push_back(labels[neighbors[j]]);
				}
				labels[i] = *std::min_element(n_labels.begin(), n_labels.end());
				unionLabels(links, n_labels);
			} else {
				labels[i] = label_num;
				links.push_back(new Set(label_num));
				label_num ++;
			}
		} else {
			labels[i] = 0;
		}
	}
	//Init balls
	for(int i = 0; i <= label_num; ++i) {
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
		if(color_fit(color_threshold,template_hue,px)
				&& im.isValid(i)
				&& (int)links.size() > labels[i]
				&& labels[i] > 0){
			labels[i] = links[labels[i]]->findS()->get();
			//Add ball data
			blobs[labels[i]].x += x;
			blobs[labels[i]].y += y;
			blobs[labels[i]].num_px ++;
			//Update t,b,l,r
			if(x > blobs[labels[i]].r.x) {
				blobs[labels[i]].r.x = x;
				blobs[labels[i]].r.y = y;
				blobs[labels[i]].valid = 1;
				//If == then not our diamond
			} else if(x == blobs[labels[i]].r.x){
				blobs[labels[i]].valid = 0;
			}
			if(x < blobs[labels[i]].l.x) {
				blobs[labels[i]].l.x = x;
				blobs[labels[i]].l.y = y;
				blobs[labels[i]].valid = 1;
			} else if(x == blobs[labels[i]].l.x){
				blobs[labels[i]].valid = 0;
			}
			if(y > blobs[labels[i]].t.y) {
				blobs[labels[i]].t.x = x;
				blobs[labels[i]].t.y = y;
				blobs[labels[i]].valid = 1;
			} else if(y == blobs[labels[i]].t.y){
				blobs[labels[i]].valid = 0;
			}
			if(y < blobs[labels[i]].b.y) {
				blobs[labels[i]].b.x = x;
				blobs[labels[i]].b.y = y;
				blobs[labels[i]].valid = 1;
			} else if(y == blobs[labels[i]].b.y){
				blobs[labels[i]].valid = 0;
			}
		}
	}

	//Filter out 
	//int err = 2;
	//int largest_idx = -1;
	int most_px    = 0;
	//printf("%d possible diamonds\n",label_num);
	int final_num_blobs = 0;
	for(int i = 1; i < label_num; ++i) {
		//Assert right num_pxs
		int right_num_pxs = (blobs[i].num_px >= min_pxs &&
							 blobs[i].num_px <= MAX_PXS);

		if(right_num_pxs) {
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
			final_labels.push_back(i);
			/*
			printf("%d passed, x: %f, y: %f, num_pxs: %d\n",i,
					temp_blobs[final_num_blobs].x,
					temp_blobs[final_num_blobs].y,
					temp_blobs[final_num_blobs].num_px);
					*/
			final_num_blobs ++;
		}
	}
	//Mark objects
	for(size_t i = 0; i < im.size(); ++i) {
		int t_label = labels[i];
		//bool set = false;
		if(color_fit(color_threshold,template_hue,im.get(i))) {
			for(int z = 0; z < final_num_blobs; ++z) {
				if(t_label == final_labels[z]) {
					//set = true;
					im.set(i,show_px);
					break;
				}
			}
		}
		//Show off color
		/*
		if(!set && t_label) {
			im.set(i,0xffff0000);
		}
		*/
	}
	//Order final_balls, so that largest is 0th
	/*
	if(largest_idx != -1) {
		final_balls[0] = temp_balls[largest_idx];
	}
	int count = 1;
	for(i = 0; i < final_num_balls; ++i) {
		if(i == largest_idx) {
			continue;
		}
		final_balls[count ++] = temp_balls[i];
	}
	*/
	//Clean up
	for(size_t i = 0; i < links.size(); ++i) {
		delete links[i];
	}
	return temp_blobs;
}

