//
// Created by huangxh on 20-7-31.
//

#ifndef COVERAGEPLANNER_COVERAGE_PLANNER_H
#define COVERAGEPLANNER_COVERAGE_PLANNER_H

#include <vector>
#include <iostream>
#include <math.h>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Surface_sweep_2_algorithms.h>
#include <CGAL/squared_distance_2.h>

#include "cgal_comm.h"
#include "decomposition.h"
#include "sweep.h"

class MouseParams{
public:
    MouseParams(const cv::Mat& img_){
        img = img_.clone();
        point = Point_2(-1, -1);
    }
    Point_2 point;
    cv::Mat img;
};

void onMouseHandle(int event, int x, int y, int flags, void* params)
{
    MouseParams* mp = (MouseParams*)params;
    switch (event)
    {
        case cv::EVENT_LBUTTONDOWN:
        {
            cv::circle(mp->img, cv::Point(x,y), 3, cv::Scalar(0, 64, 255), -1);
            cv::imshow("select start point", mp->img);
            std::cout<<"<start point> x: "<<x<<", y: "<<y<<std::endl;
            mp->point = Point_2(x, y);
            break;
        }
        default:
        {
            break;
        }
    }
}

Point_2 getStartingPoint(cv::Mat& img)
{
    MouseParams params(img);
    params.img = img.clone();

    cv::namedWindow("select start point", cv::WINDOW_AUTOSIZE);
    cv::imshow("select start point", params.img);
    cv::setMouseCallback("select start point", onMouseHandle, (void*)&(params));
    cv::waitKey();
    Point_2 point = params.point;

    img = params.img.clone();

    cv::destroyWindow("select start point");
    return point;
}

class CellNode
{
public:
    CellNode()
    {
        isVisited = false;
        isCleaned = false;
        parentIndex = INT_MAX;
        cellIndex = INT_MAX;
    }
    bool isVisited;
    bool isCleaned;

    int parentIndex;
    std::deque<int> neighbor_indices;

    int cellIndex;
};

std::vector<CellNode> calculateDecompositionAdjacency(const std::vector<Polygon_2>& decompositions) {

    std::vector<CellNode> polygon_adj_graph(decompositions.size());
    for (size_t i = 0; i < decompositions.size() - 1; ++i) {
        polygon_adj_graph[i].cellIndex = i;
        for (size_t j = i + 1; j < decompositions.size(); ++j) {
            PolygonWithHoles joined;
            if (CGAL::join(decompositions[i], decompositions[j], joined)) {
                polygon_adj_graph[i].neighbor_indices.emplace_back(j);
                polygon_adj_graph[j].neighbor_indices.emplace_back(i);
            }
        }
    }
    polygon_adj_graph.back().cellIndex = decompositions.size()-1;

    return polygon_adj_graph;
}

// DFS
void walkThroughGraph(std::vector<CellNode>& cell_graph, int cell_index, int& unvisited_counter, std::deque<CellNode>& path)
{
    if(!cell_graph[cell_index].isVisited){
        cell_graph[cell_index].isVisited = true;
        unvisited_counter--;
    }
    path.emplace_front(cell_graph[cell_index]);

//    for debugging
//    std::cout<< "cell: " <<cell_graph[cell_index].cellIndex<<std::endl;
//

    CellNode neighbor;
    int neighbor_idx = INT_MAX;

    for(int i = 0; i < cell_graph[cell_index].neighbor_indices.size(); i++){
        neighbor = cell_graph[cell_graph[cell_index].neighbor_indices[i]];
        neighbor_idx = cell_graph[cell_index].neighbor_indices[i];
        if(!neighbor.isVisited){
            break;
        }
    }

    // unvisited neighbor found
    if(!neighbor.isVisited){
        cell_graph[neighbor_idx].parentIndex = cell_graph[cell_index].cellIndex;
        walkThroughGraph(cell_graph, neighbor_idx, unvisited_counter, path);
    }
    // unvisited neighbor not found
    else{
        // cannot go on back-tracking
        if (cell_graph[cell_index].parentIndex == INT_MAX){
            return;
        }else if(unvisited_counter == 0){
            return;
        }else{
            walkThroughGraph(cell_graph, cell_graph[cell_index].parentIndex, unvisited_counter, path);
        }
    }
}

std::deque<int> getTravellingPath(const std::vector<CellNode>& cell_graph, int first_cell_index)
{
    std::deque<int> travelling_path;

    std::deque<CellNode> _cell_path;
    std::vector<CellNode> _cell_graph = cell_graph;

    if(_cell_graph.size()==1){
        travelling_path.emplace_back(0);
    }else{
        int unvisited_counter = _cell_graph.size();
        walkThroughGraph(_cell_graph, first_cell_index, unvisited_counter, _cell_path);
        std::reverse(_cell_path.begin(), _cell_path.end());
    }

    for(auto& cell : _cell_path){
        travelling_path.emplace_back(cell.cellIndex);
    }

    return travelling_path;
}

std::vector<std::map<int, std::list<Point_2 >>> calculateCellIntersections(std::vector<Polygon_2>& decompositions, std::vector<CellNode>& cell_graph){

    std::vector<std::map<int, std::list<Point_2 >>> cell_intersections(cell_graph.size());

    for(size_t i = 0; i < cell_graph.size(); ++i){
        for(size_t j = 0; j < cell_graph[i].neighbor_indices.size(); ++j){
            std::list<Point_2> pts;
            for(auto m = decompositions[i].edges_begin(); m != decompositions[i].edges_end(); ++m){
                for(auto n = decompositions[cell_graph[i].neighbor_indices[j]].edges_begin();
                    n != decompositions[cell_graph[i].neighbor_indices[j]].edges_end();
                    ++n){
                    Segment_2 segments[] = {*m, *n};
                    CGAL::compute_intersection_points(segments, segments+2, std::back_inserter(pts));
                }
            }

            for(auto p = decompositions[i].vertices_begin(); p != decompositions[i].vertices_end(); ++p){
                for(auto q = decompositions[cell_graph[i].neighbor_indices[j]].vertices_begin(); q != decompositions[cell_graph[i].neighbor_indices[j]].vertices_end(); ++q){
                    if(CGAL::to_double(p->x())==CGAL::to_double(q->x()) && CGAL::to_double(p->y())==CGAL::to_double(q->y())){
                        pts.insert(pts.end(), *p);
                    }
                }
            }

            auto verbose = std::unique(pts.begin(), pts.end());
            pts.erase(verbose, pts.end());
            cell_intersections[i].insert(std::make_pair(cell_graph[i].neighbor_indices[j], pts));
            cell_intersections[cell_graph[i].neighbor_indices[j]].insert(std::make_pair(i, pts));
        }
    }

    return cell_intersections;
}

Point_2 findNextGoal(const Point_2& start, const Point_2& goal, const std::list<Point_2>& candidates){
    double min_cost = DBL_MAX;
    double cost;
    Point_2 next_point = start;
    Segment_2 seg_from_start, seg_to_goal;
    for(auto point = candidates.begin(); point != candidates.end(); ++point){
        seg_from_start = Segment_2(start, *point);
        seg_to_goal = Segment_2(*point, goal);
        cost = CGAL::to_double(seg_from_start.squared_length())+CGAL::to_double(seg_to_goal.squared_length());
        if(cost < min_cost){
            min_cost = cost;
            next_point = *point;
        }
    }
    return next_point;
}

bool doReverseNextSweep(const Point_2& curr_point, const std::vector<Point_2>& next_sweep){
    return CGAL::to_double(CGAL::squared_distance(curr_point, next_sweep.front())) > CGAL::to_double(CGAL::squared_distance(curr_point, next_sweep.back()));
}

std::vector<Point_2> getShortestPath(const Polygon_2& polygon, const Point_2& start, const Point_2& goal){
    polygon_coverage_planning::visibility_graph::VisibilityGraph vis_graph(polygon);
    std::vector<Point_2> shortest_path;
    polygon_coverage_planning::calculateShortestPath(vis_graph, start, goal, &shortest_path);
    return shortest_path;
}

int getCellIndexOfPoint(const std::vector<Polygon_2>& decompositions, const Point_2& point){
    int index = -1;
    for(int i = 0; i < decompositions.size(); i++){
        if(polygon_coverage_planning::pointInPolygon(decompositions[i], point)){
            index = i;
            break;
        }
    }
    return index;
}

#endif //COVERAGEPLANNER_COVERAGE_PLANNER_H
