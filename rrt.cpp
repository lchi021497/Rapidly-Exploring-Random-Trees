#include <iostream>
#include <cstdio>
#include <random>
#include <SFML/Graphics.hpp>
#include "geometry.h"
#include <stdlib.h>
#include <time.h>
#include <omp.h>
#include <chrono> 
#include "kdtree.hpp"
#include <algorithm>
#include <atomic>
 

// #define ON_MAC true

using namespace std ; 

const int WIDTH = 1200 ;
const int HEIGHT = 800 ;
const int RADIUS = 5 ; 
const double GOAL_SAMPLING_PROB = 0.05;
const double INF = 1e18;

const double JUMP_SIZE = (WIDTH/100.0 * HEIGHT/100.0)/1.50;
const double DISK_SIZE = JUMP_SIZE ; // Ball radius around which nearby points are found 

int whichRRT = 1 ; 

vector < Polygon > obstacles ; 
Point start, stop ; 
int obstacle_cnt = 1 ;

int nodeCnt = 0;
Kdtree::kdtree_node* goalNode; 
vector <sf::ConvexShape> polygons ;
sf::CircleShape startingPoint, endingPoint ; 
bool pathFound = 0 ;

void getInput() {
	// cout << "NOTE:" << endl ; 
	// cout << "Height of screen: " << HEIGHT << " pixels." ;
	// cout << " Width of screeen: " << WIDTH << " pixels." << endl ;
	// cout << "Maximum distance by which algorithm jumps from one point to another: " << JUMP_SIZE << " units" << endl ;
	// cout << "If you would like to change of any of these, please make modifications in code" << endl ; 
	// cout << "Please provide your inputs keeping this in mind. " << endl << endl ;

	// cout << "Which type of RRT would you like to watch? 1 for RRT, 2 for RRT*, 3 for Anytime RRT" << endl ;
	// cin >> whichRRT ; 
	// cout << "Input co-ordinates of starting and ending point respectively in this format X1 Y1 X2 Y2" << endl ;
	// cin >> start.x >> start.y >> stop.x >> stop.y ;
	// cout << "How many obstacles?" << endl ; 
	// cin >> obstacle_cnt ; 
	
	// obstacles.resize(obstacle_cnt); 
	// int pnts = 0 ; Point pnt ; 
	// vector < Point > poly ; 
	
	// for(int i = 0; i < obstacle_cnt; i++) {
	// 	poly.clear();
	// 	cout << "How many points in " << i+1 << "th polygon?" << endl ; 
	// 	cin >> pnts ; 
	// 	poly.resize(pnts);

	// 	cout << "Input co-ordinates of " << i+1 << "th polygon in clockwise order" << endl ;
	// 	for(int j = 0; j < pnts; j++) {
	// 		cin >> pnt.x >> pnt.y ; 
	// 		obstacles[i].addPoint(pnt);
	// 	}
	// }

	// whichRRT = 2;
	// start.x = 50;
	// start.y = 50;
	// stop.x = 1150;
	// stop.y = 750;

	// obstacle_cnt = 0;
	// obstacles.resize(obstacle_cnt); 

	Point pnt;

	whichRRT = 2;
	start.x = 600;
	start.y = 400;
	stop.x = 50;
	stop.y = 400;

	obstacle_cnt = 6;
	obstacles.resize(obstacle_cnt); 

	pnt.x = 300; pnt.y = 450;
	obstacles[0].addPoint(pnt);
	pnt.x = 300; pnt.y = 500;
	obstacles[0].addPoint(pnt);
	pnt.x = 950; pnt.y = 500;
	obstacles[0].addPoint(pnt);
	pnt.x = 950; pnt.y = 450;
	obstacles[0].addPoint(pnt);

	pnt.x = 900; pnt.y = 500;
	obstacles[1].addPoint(pnt);
	pnt.x = 950; pnt.y = 500;
	obstacles[1].addPoint(pnt);
	pnt.x = 950; pnt.y = 300;
	obstacles[1].addPoint(pnt);
	pnt.x = 900; pnt.y = 300;
	obstacles[1].addPoint(pnt);

	pnt.x = 300; pnt.y = 350;
	obstacles[2].addPoint(pnt);
	pnt.x = 950; pnt.y = 350;
	obstacles[2].addPoint(pnt);
	pnt.x = 950; pnt.y = 300;
	obstacles[2].addPoint(pnt);
	pnt.x = 300; pnt.y = 300;
	obstacles[2].addPoint(pnt);


	pnt.x = 1100; pnt.y = 650;
	obstacles[3].addPoint(pnt);
	pnt.x = 1100; pnt.y = 600;
	obstacles[3].addPoint(pnt);
	pnt.x = 100; pnt.y = 600;
	obstacles[3].addPoint(pnt);
	pnt.x = 100; pnt.y = 650;
	obstacles[3].addPoint(pnt);

	pnt.x = 100; pnt.y = 650;
	obstacles[4].addPoint(pnt);
	pnt.x = 100; pnt.y = 150;
	obstacles[4].addPoint(pnt);
	pnt.x = 150; pnt.y = 150;
	obstacles[4].addPoint(pnt);
	pnt.x = 150; pnt.y = 650;
	obstacles[4].addPoint(pnt);

	pnt.x = 100; pnt.y = 150;
	obstacles[5].addPoint(pnt);
	pnt.x = 100; pnt.y = 100;
	obstacles[5].addPoint(pnt);
	pnt.x = 1100; pnt.y = 100;
	obstacles[5].addPoint(pnt);
	pnt.x = 1100; pnt.y = 150;
	obstacles[5].addPoint(pnt);

	// whichRRT = 2;
	// start.x = 50;
	// start.y = 50;
	// stop.x = 1150;
	// stop.y = 50;

	// obstacle_cnt = 1;
	// obstacles.resize(obstacle_cnt); 

	// pnt.x = 550; pnt.y = 0;
	// obstacles[0].addPoint(pnt);
	// pnt.x = 650; pnt.y = 0;
	// obstacles[0].addPoint(pnt);
	// pnt.x = 650; pnt.y = 700;
	// obstacles[0].addPoint(pnt);
	// pnt.x = 550; pnt.y = 700;
	// obstacles[0].addPoint(pnt);

	
	// whichRRT = 2;
	// start.x = 100;
	// start.y = 100;
	// stop.x = 100;
	// stop.y = 700;

	// obstacle_cnt = 1;
	// obstacles.resize(obstacle_cnt); 

	// pnt.x = 0; pnt.y = 450;
	// obstacles[0].addPoint(pnt);
	// pnt.x = 1150; pnt.y = 450;
	// obstacles[0].addPoint(pnt);
	// pnt.x = 1150; pnt.y = 350;
	// obstacles[0].addPoint(pnt);
	// pnt.x = 0; pnt.y = 350;
	// obstacles[0].addPoint(pnt);


}

// Prepares SFML objects of starting, ending point and obstacles 
void prepareInput() {
	// Make starting and ending point circles ready 
	startingPoint.setRadius(RADIUS); endingPoint.setRadius(RADIUS); 
    startingPoint.setFillColor(sf::Color(208, 0, 240)); endingPoint.setFillColor(sf::Color::Blue);
#if defined(ON_MAC)
	sf::Vector2f start_position(start.x, start.y);
    sf::Vector2f end_position(stop.x, stop.y);
    startingPoint.setPosition(start_position); endingPoint.setPosition(end_position);
    sf::Vector2f origin1(RADIUS/2, RADIUS/2);
    sf::Vector2f origin2(RADIUS/2, RADIUS/2);
    startingPoint.setOrigin(origin1); endingPoint.setOrigin(origin2);
#else
    startingPoint.setPosition(start.x, start.y); endingPoint.setPosition(stop.x, stop.y);
    startingPoint.setOrigin(RADIUS/2, RADIUS/2); endingPoint.setOrigin(RADIUS/2, RADIUS/2);
#endif

    // Prepare polygon of obstacles 
	polygons.resize(obstacle_cnt);
	for(int i = 0; i < obstacle_cnt; i++) {
		polygons[i].setPointCount(obstacles[i].pointCnt); 
		polygons[i].setFillColor(sf::Color(89, 87, 98)); 
		for(int j = 0; j < obstacles[i].pointCnt; j++) 
			polygons[i].setPoint(j, sf::Vector2f(obstacles[i].points[j].x, obstacles[i].points[j].y));
	}
}

void draw_tree(sf::RenderWindow& window, Kdtree::kdtree_node* root) {
	sf::Vertex line[2]; 

	if (root->par_cost.load()->parent != nullptr) {
		line[0] = sf::Vertex(sf::Vector2f(root->par_cost.load()->parent->point[0], root->par_cost.load()->parent->point[1]));
		line[1] = sf::Vertex(sf::Vector2f(root->point[0], root->point[1]));
#if defined(ON_MAC)
		window.draw(line, 2, sf::PrimitiveType::Lines);
#else
		window.draw(line, 2, sf::Lines);
#endif

	}
	if(root->loson) {
		draw_tree(window, root->loson);
	}
	if(root->hison) {
		draw_tree(window, root->hison);
	}

}

void draw(sf::RenderWindow& window, Kdtree::KdTree &kdtree) {
	sf::Vertex line[2]; sf::CircleShape nodeCircle;

	// Uncomment if circular nodes are to be drawn 
	/*
	for(auto& node: nodes) {
		nodeCircle.setRadius(RADIUS/2.5); // nodeCircle.setRadius(min(2.0, RADIUS/2.0));
		nodeCircle.setOrigin(RADIUS/2.5, RADIUS/2.5); 
		nodeCircle.setFillColor(sf::Color(0, 255, 171)); nodeCircle.setPosition(node.x, node.y); 
		window.draw(nodeCircle);
	}


	*/

	// Draw obstacles 
	for(auto& poly : polygons) window.draw(poly);

	draw_tree(window, kdtree.root);

	window.draw(startingPoint); window.draw(endingPoint);

	// If destination is reached then path is retraced and drawn 
	if(pathFound) {

		Kdtree::kdtree_node *ptr = goalNode;

		std::vector<int> path; 
		while(ptr->par_cost.load()->parent != nullptr) {

			if (std::find(path.begin(), path.end(), ptr->index) != path.end()) {
				printf("found cycle of length %ld", path.size());
				goalNode = nullptr;
				pathFound = 0;
				break;
			}

			path.push_back(ptr->index);
			line[0] = sf::Vertex(sf::Vector2f(ptr->par_cost.load()->parent->point[0], ptr->par_cost.load()->parent->point[1]));
			line[1] = sf::Vertex(sf::Vector2f(ptr->point[0], ptr->point[1]));
			line[0].color = line[1].color = sf::Color::Red; // orange color 

			ptr = ptr->par_cost.load()->parent;
#if defined(ON_MAC)
			window.draw(line, 2, sf::PrimitiveType::Lines);
			
#else
			window.draw(line, 2, sf::Lines);
#endif
		}

	}
}

template <typename T> // Returns a random number in [low, high] 
T randomCoordinate(T low, T high){
    random_device random_device;
    //default_random_engine engine;
    mt19937 engine{random_device()};
    uniform_real_distribution<double> dist(low, high);
    return dist(engine);
}

// Returns true if the line segment ab is obstacle free
bool isEdgeObstacleFree(Point a, Point b) {
    for(auto& poly: obstacles)
        if(lineSegmentIntersectsPolygon(a, b, poly))
        	return false ; 
    return true ; 
}

// Returns a random point with some bias towards goal 
Point pickRandomPoint() {
    double random_sample = randomCoordinate(0.0, 1.0); 
    if((random_sample - GOAL_SAMPLING_PROB) <= EPS and !pathFound) return stop + Point(RADIUS, RADIUS) ;
	return Point(randomCoordinate(0, WIDTH), randomCoordinate(0, HEIGHT)); 
}

bool checkDestinationReached(Kdtree::kdtree_node *newNode, Kdtree::KdTree &kdtree) {
	sf::Vector2f position = endingPoint.getPosition(); 

	Kdtree::kdtree_node *parent = newNode->par_cost.load()->parent;

	if (parent==nullptr)
		return false;

	Point newNodePoint = Point(newNode->point[0], newNode->point[1]); 
	Point parentPoint = Point(newNode->par_cost.load()->parent->point[0], newNode->par_cost.load()->parent->point[1]);

	if(checkCollision(parentPoint, newNodePoint, Point(position.x, position.y), RADIUS)) {
		pathFound = 1 ;

		Point goalPoint({stop.x, stop.y});

		// goalNode = newNode;
		// cout << "tree: Reached!! With a distance of " << newNode->par_cost.load()->cost << " units. " << endl << endl ;
		
		double cost_node = newNode->par_cost.load()->cost + distance(newNodePoint, goalPoint);
		goalNode = kdtree.insert(Kdtree::CoordPoint ({stop.x, stop.y}), -1, cost_node, newNode);
		cout << "tree: Reached!! With a distance of " << cost_node << " units. " << endl << endl ;
		return true; 
	}
	return false;
}

/*  Rewires the parents of the tree greedily starting from 
	the new node found in this iterationsation as the parent */
void rewire(Kdtree::KdTreeNodeVec nearby, Kdtree::kdtree_node *newNode) {

	for (auto kdnode: nearby) {

		if (kdnode->index == newNode->index) continue;
		if (kdnode->par_cost.load()->parent == nullptr) continue;

		Kdtree::kdtree_node *par = newNode; 
		Point parPnt = Point(par->point[0], par->point[1]);

		Kdtree::kdtree_node *cur = kdnode; 
		Point curPnt = Point(cur->point[0], cur->point[1]);

		if (!isEdgeObstacleFree(curPnt, parPnt)) continue;


		while ( ((par->par_cost.load()->cost + distance(parPnt, curPnt)) - cur->par_cost.load()->cost) <= EPS) {


			Kdtree::rewire_par_cost *old = cur->par_cost;

			Kdtree::rewire_par_cost *newValue = new Kdtree::rewire_par_cost(); 

			newValue->parent = par; 
			newValue->cost = par->par_cost.load()->cost + distance(parPnt, curPnt); 
			
			if ((cur->par_cost).compare_exchange_weak(old, newValue, std::memory_order_release, std::memory_order_relaxed)) {
				delete old;
				break;
			}
			else {
				delete newValue;
			}

		}

	}
}


/*	Runs one iteration of RRT depending on user choice 
	At least one new node is added on the screen each iteration. */
void RRT(Kdtree::KdTree &kdtree) {
	Point newPoint, nearestPoint, nextPoint ; bool updated = false ; int cnt = 0 ; 
	double minCost = INF;

	std::chrono::steady_clock::time_point begin;
	std::chrono::steady_clock::time_point end; 

	int index = nodeCnt + omp_get_thread_num();
	
	while(!updated) {

		newPoint = pickRandomPoint(); 
   		Kdtree::CoordPoint queryPoint({newPoint.x, newPoint.y});

		// Find nearest point to the newPoint such that the next node 
		// be added in graph in the (nearestPoint, newPoint) while being obstacle free

		nearestPoint = Point(kdtree.root->point[0], kdtree.root->point[1]);

		int num_neighbors = 1;

		begin = std::chrono::steady_clock::now();

		Kdtree::kdtree_node* nearest_neighbor; 

		Kdtree::KdTreeNodeVec res;
		kdtree.k_nearest_neighbors(queryPoint, num_neighbors, &res);

		for (auto kdnode: res) {

			Point pnt(kdnode->point[0], kdnode->point[1]);

			// if (kdnode->par_cost.load()->parent != nullptr) {
			// 	Point parentPnt(kdnode->par_cost.load()->parent->point[0], kdnode->par_cost.load()->parent->point[1]);

			// 	if(pathFound and randomCoordinate(0.0, 1.0) < 0.25) // Recalculate cost once in a while 
			// 		kdnode->par_cost.load()->cost = kdnode->par_cost.load()->parent->par_cost.load()->cost +  distance(parentPnt, pnt);
			// }
			nearestPoint = pnt; 
			nearest_neighbor = kdnode;

		}
		
		nextPoint = stepNear(nearestPoint, newPoint, randomCoordinate(0.3, 1.0) * JUMP_SIZE);

		if(!isEdgeObstacleFree(nearestPoint, nextPoint)) continue ; 

		if( (whichRRT == 1) or (!pathFound and whichRRT == 3)) {
			// This is where we don't do any RRT* optimization part 
			updated = true ;
			double cost_node = nearest_neighbor->par_cost.load()->cost + distance(nearestPoint, nextPoint);
			Kdtree::kdtree_node *newNode = kdtree.insert(Kdtree::CoordPoint ({nextPoint.x, nextPoint.y}), index, cost_node, nearest_neighbor);

			if(!pathFound) checkDestinationReached(newNode, kdtree);

			continue ; 
		}


		double minCost; 
		Kdtree::KdTreeNodeVec nearby;

    	kdtree.range_nearest_neighbors(Kdtree::CoordPoint({nextPoint.x, nextPoint.y}), DISK_SIZE, &nearby);
		minCost = nearest_neighbor->par_cost.load()->cost + distance(nearestPoint, nextPoint);
		Kdtree::kdtree_node* optim_parent = nearest_neighbor; 

		for (auto kdnode:nearby) {

			Point nodePnt(kdnode->point[0], kdnode->point[1]);

			if (!isEdgeObstacleFree(nodePnt, nextPoint)) continue;

			if (((kdnode->par_cost.load()->cost + distance(nodePnt, nextPoint)) - minCost) <= EPS) {
				minCost = kdnode->par_cost.load()->cost + distance(nodePnt, nextPoint); 
				optim_parent = kdnode;
			}
		}
	
		Kdtree::kdtree_node *newNode = kdtree.insert(Kdtree::CoordPoint ({nextPoint.x, nextPoint.y}), index, minCost, optim_parent);

		updated = true ; 


		if(!pathFound) checkDestinationReached(newNode, kdtree);

		// #pragma omp critical
		rewire(nearby, newNode);
		
		
	}
}

int main(int argc, char* argv[]) {

	Kdtree::CoordPoint min_point({0, 0});
	Kdtree::CoordPoint max_point({WIDTH, HEIGHT});
	size_t num_dim = 2;
	Kdtree::KdTree kdtree(min_point, max_point, num_dim);

	std::chrono::steady_clock::time_point begin;
	std::chrono::steady_clock::time_point end; 

	// whichRRT = 2; 
	// start.x = 0;
	// start.y = 0;
	// stop.x = 500;
	// stop.y = 500;
	// obstacle_cnt = 0; 
	getInput(); 
	prepareInput();
#if defined(ON_MAC)
	sf::Vector2u dimensions(WIDTH, HEIGHT);
    sf::RenderWindow window(sf::VideoMode(dimensions, 1), "Basic Anytime RRT");
#else
    sf::RenderWindow window(sf::VideoMode(WIDTH, HEIGHT), "Basic Anytime RRT");
#endif

	nodeCnt = 1; int iterations = 0 ; 

	kdtree.insert(Kdtree::CoordPoint ({start.x, start.y}), 0, 0, nullptr);
    sf::Time delayTime = sf::milliseconds(5);

    cout << endl << "Starting node is in Pink and Destination node is in Blue" << endl << endl ; 

    while (window.isOpen())
    {

	// while(true) {

        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
            {
            	window.close();
            	return 0; exit(0);
            }
        }

		int num_threads = 1;
		if (argc == 2)
			num_threads = atoi(argv[1]);

		#pragma omp parallel for
		for (int i = 0; i < num_threads; i++) {

			RRT(kdtree);

		}
		iterations+=num_threads;
		nodeCnt += num_threads;

		if(iterations % 500 == 0) {
			cout << "Iterations: " << iterations << endl ; 
			if(!pathFound) cout << "Not reached yet :( " << endl ;
			else cout << "Shortest distance till now: " << goalNode->par_cost.load()->cost << " units." << endl ;
			cout << endl ;
		}

		sf::sleep(delayTime);
		window.clear();
		draw(window, kdtree); 
        window.display();
    }
}

/* SOME SAMPLE INPUTS ARE SHOWN BELOW (only cin part) without any RRT preference */ 

/*
100 70
600 400
2
4
200 480
200 100
250 100
250 480
5
400 0
300 100
350 250
450 250
500 100
*/

/*
50 50
750 180
3
4
100 0
200 0
200 400
100 400
4
250 200
350 200
350 600
250 600
4
400 50
550 50
550 250
400 250
*/

/*
50 50
750 580
3
4
100 0
200 0
200 400
100 400
4
250 200
350 200
350 600
250 600
4
400 32
700 32
700 575
400 575
*/

/*
190 30
660 500
5
3
200 50
200 200
350 200
3
220 50
370 50
370 200
3
400 250
400 450
600 450
3
430 250
630 250
630 450
3
640 470
640 550
680 550
*/

/*
190 55
660 500
9
4
740 360 
750 350
680 540
670 530
3
710 290
780 350
630 380
3
520 450
620 540
349 580
4
450 70
700 70
700 240 
450 240
3
200 50
200 200
350 200
3
220 50
370 50
370 200
3
400 250
400 450
600 450
3
430 250
630 250
630 450
3
640 470
640 550
680 550
*/
