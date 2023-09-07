#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <map>
#include <string>
#include <Eigen/Dense>
#include <dlib/optimization/max_cost_assignment.h>
#include "Ellipse.h"

using namespace std;

namespace ORB_SLAM2
{
    class Object;

    struct Attribute {
        int object_id;
        int label;
        float confidence;
        float hue;
        Eigen::Vector4d bbox;
        Ellipse ell;
        Object* obj;
    };

    class Graph {
        public:
            Graph();

            Graph(vector<pair<int,int>> edge_list, vector<pair<int,int>> node_labels);

            void add_node(int node_id, int label, float confidence=0.0f, float hue=0.0f, Eigen::Vector4d bbox=Eigen::Vector4d::Zero(), Ellipse ell=Ellipse());

            void add_edge(int node1, int node2, float weight=1.0f);

            bool has_edge(int node1, int node2) {
                return edges.count(make_pair(node1,node2)) > 0;
            }

            bool has_node(int n){
                return nodes.count(n)>0;
            }

            vector<int> get_neighbours(int node_id) {
                return nodes[node_id];
            }

            void compute_catagory_statistics();

            void compute_feature_vectors();

            Eigen::VectorXd compute_feature_vector_node(int node_id);

            double find_matches_score(map<int, std::vector<pair<int, Eigen::VectorXd>>> feature_vectors_another, 
                              std::vector<tuple<pair<int, int>, double>>& matches, size_t N_another, double thres=0.1);

            bool has_false(std::vector<bool> used){
                return std::any_of(used.begin(), used.end(), [](bool b){return !b;});
            }

            
            map<int, vector<int>> nodes;
            map<pair<int,int>, float> edges;
            map<int, Attribute> attributes;
            map<int, std::vector<pair<int, Eigen::VectorXd>>> feature_vectors; 
            Eigen::VectorXd category_ids_statistics;
    };

} //namespace ORB_SLAM

#endif // GRAPH_H
