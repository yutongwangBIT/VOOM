/**
* This file is part of OA-SLAM.
*
* Copyright (C) 2022 Matthieu Zins <matthieu.zins@inria.fr>
* (Inria, LORIA, Université de Lorraine)
* OA-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* OA-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with OA-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/


#include "Graph.h"


namespace ORB_SLAM2 
{
    Graph::Graph(){
        category_ids_statistics = Eigen::VectorXd::Zero(80);
    }

    Graph::Graph(vector<pair<int,int>> edge_list, vector<pair<int,int>> node_labels) {
        for (auto& [node_id,label] : node_labels) {
            add_node(node_id,label);
        }
        category_ids_statistics = Eigen::VectorXd::Zero(80);
        for (auto& [node1,node2] : edge_list) {
            add_edge(node1,node2);
        }
        compute_feature_vectors(); 
    }

    void Graph::add_node(int node_id, int label, float confidence, float hue, Eigen::Vector4d bbox, Ellipse ell) {
        Attribute attribute = {-1, label, confidence, hue, bbox, ell, nullptr};
        nodes[node_id] = vector<int>();
        attributes[node_id] = attribute;
    }

    void Graph::add_edge(int node1, int node2, float weight) {
        if (node1 > node2) {
            swap(node1,node2);
        }
        if (!has_edge(node1,node2)){
            edges[make_pair(node1,node2)] = weight;
            nodes[node1].push_back(node2);
            nodes[node2].push_back(node1);
        }
    }

    void Graph::compute_catagory_statistics(){
        category_ids_statistics.setZero();
        for(auto& node : nodes)
            category_ids_statistics[attributes[node.first].label] += 1;
        for(size_t i=0; i<category_ids_statistics.size(); i++){
            category_ids_statistics[i] /= nodes.size();
        } 
    } 

    void Graph::compute_feature_vectors() {
        compute_catagory_statistics();
        for (auto& [node_id, neighbours] : nodes) {
            Eigen::VectorXd feature_vector(80);
            feature_vector.setZero(); 
            for (int neighbour : neighbours) {
                feature_vector[attributes[neighbour].label] += 1-category_ids_statistics[attributes[neighbour].label];//1.0f;
            }
            feature_vectors[attributes[node_id].label].push_back(make_pair(node_id, feature_vector));
        }
    }

    Eigen::VectorXd Graph::compute_feature_vector_node(int node_id){
        Eigen::VectorXd feature_vector(80);
        feature_vector.setZero(); 
        for (int neighbour : nodes[node_id]) {
            feature_vector[attributes[neighbour].label] += 1.0f;
        }
        return feature_vector;
    }

    double Graph::find_matches_score(map<int, std::vector<pair<int, Eigen::VectorXd>>> feature_vectors_another, 
                              std::vector<tuple<pair<int, int>, double>>& matches, size_t N_another, double thres) {
        double sum_score = 0.0f;
        int count = 0;
        for (auto& [label, feature_vectors_per_label1] : feature_vectors){
            auto feature_vectors_per_label2 = feature_vectors_another[label];
            if(feature_vectors_per_label2.empty()) continue;
            if(feature_vectors_per_label1.size()>1 || feature_vectors_per_label2.size()>1){
                std::vector<std::vector<double>> match_graph(feature_vectors_per_label1.size(), 
                        std::vector<double>(feature_vectors_per_label2.size(),0));
                for(size_t j=0; j<feature_vectors_per_label1.size(); j++){
                    Eigen::VectorXd feature1 = feature_vectors_per_label1[j].second;
                    for(size_t k=0; k<feature_vectors_per_label2.size(); k++){
                        Eigen::VectorXd feature2 = feature_vectors_per_label2[k].second;
                        match_graph[j][k] = feature1.dot(feature2)/(feature1.norm() * feature2.norm());
                    }
                }

                //std::vector<int> match(feature_vectors_per_label1.size(),-1);
                std::vector<bool> used1(feature_vectors_per_label1.size(),false);
                std::vector<bool> used2(feature_vectors_per_label2.size(),false);
                double global_max = 1.0;
                while(global_max>thres && has_false(used1) && has_false(used2) ){
                    double match_max = 0.0;
                    std::pair<int, int> pair;
                    for(size_t j=0; j<feature_vectors_per_label1.size(); j++){
                        if(used1[j]) continue;
                        for(size_t k=0; k<feature_vectors_per_label2.size(); k++){
                            if(match_graph[j][k]>match_max && !used2[k]){
                                match_max = match_graph[j][k];
                                pair = make_pair(j, k);
                            }
                        }
                    }
                    if(match_max>thres){
                        matches.push_back(make_tuple(make_pair(feature_vectors_per_label1[pair.first].first,
                                                    feature_vectors_per_label2[pair.second].first), match_max));
                        sum_score += match_max;
                        count += 1;
                        //match[pair.first] = pair.second;
                        used1[pair.first] = true;
                        used2[pair.second] = true;
                    }
                    global_max = match_max;
                }
            }
            else{
                auto feature_pair1 = feature_vectors_per_label1[0];
                int node_id1 = feature_pair1.first;
                Eigen::VectorXd feature1 = feature_pair1.second;
                auto feature_pair2 = feature_vectors_per_label2[0];
                int node_id2 = feature_pair2.first;
                Eigen::VectorXd feature2 = feature_pair2.second;
                //std::cout<<"norm1:"<<feature1.norm()<<std::endl;
                //std::cout<<"norm2:"<<feature2.norm()<<std::endl; 
                double cos_sim = feature1.dot(feature2)/(feature1.norm() * feature2.norm());
                //std::cout<<"cossim:"<<cos_sim<<std::endl;    
                if (cos_sim > thres){
                    matches.push_back(make_tuple(make_pair(node_id1, node_id2), cos_sim));
                    sum_score += cos_sim;
                    count += 1;
                }
            }
        }
        int N = max(nodes.size(), N_another);
        return (double)count / (double)N;// sum_score/N;
    }

}
