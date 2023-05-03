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
 

#define ON_MAC true

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



vector < Point > nodes ; 
vector < int > parent; 
vector < double > cost ; 
int nodeCnt = 0, goalIndex = -1 ; 

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

	whichRRT = 3;
	start.x = 100;
	start.y = 100;
	stop.x = 700;
	stop.y = 500;

	obstacle_cnt = 0;
	// obstacle_cnt = 2;
	// obstacles.resize(obstacle_cnt); 

	// Point pnt;
	// pnt.x = 200; pnt.y = 480;
	// obstacles[0].addPoint(pnt);
	// pnt.x = 200; pnt.y = 100;
	// obstacles[0].addPoint(pnt);
	// pnt.x = 250; pnt.y = 100;
	// obstacles[0].addPoint(pnt);
	// pnt.x = 250; pnt.y = 480;
	// obstacles[0].addPoint(pnt);

	// pnt.x = 400; pnt.y = 0;
	// obstacles[1].addPoint(pnt);
	// pnt.x = 300; pnt.y = 100;
	// obstacles[1].addPoint(pnt);
	// pnt.x = 350; pnt.y = 250;
	// obstacles[1].addPoint(pnt);
	// pnt.x = 450; pnt.y = 250;
	// obstacles[1].addPoint(pnt);
	// pnt.x = 500; pnt.y = 100;
	// obstacles[1].addPoint(pnt);

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

void draw(sf::RenderWindow& window) {
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

	// Draw edges between nodes 
	for(int i = (int)nodes.size() - 1; i; i--) {
		Point par = nodes[parent[i]] ; 
		line[0] = sf::Vertex(sf::Vector2f(par.x, par.y));
		line[1] = sf::Vertex(sf::Vector2f(nodes[i].x, nodes[i].y));




#if defined(ON_MAC)
		window.draw(line, 2, sf::PrimitiveType::Lines);
#else
		window.draw(line, 2, sf::Lines);
#endif
	}

	window.draw(startingPoint); window.draw(endingPoint);

	// If destination is reached then path is retraced and drawn 
	if(pathFound) {
		int node = goalIndex; 
		while(parent[node] != node) {
			int par = parent[node];
			line[0] = sf::Vertex(sf::Vector2f(nodes[par].x, nodes[par].y));
			line[1] = sf::Vertex(sf::Vector2f(nodes[node].x, nodes[node].y));
			line[0].color = line[1].color = sf::Color::Red; // orange color 
#if defined(ON_MAC)
			window.draw(line, 2, sf::PrimitiveType::Lines);
			
#else
			window.draw(line, 2, sf::Lines);
#endif
			node = par ;
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

bool checkDestinationReached() {
	sf::Vector2f position = endingPoint.getPosition(); 
	if(checkCollision(nodes[parent[nodeCnt - 1]], nodes.back(), Point(position.x, position.y), RADIUS)) {
		pathFound = 1 ; 
		goalIndex = nodeCnt - 1;
		cout << "Reached!! With a distance of " << cost.back() << " units. " << endl << endl ;
		return true; 
	}
	return false;
}

bool checkDestinationReached(Kdtree::kdtree_node *newNode) {
	sf::Vector2f position = endingPoint.getPosition(); 

	Kdtree::kdtree_node *parent = newNode->par_cost.load()->parent;

	if (parent==nullptr)
		return false;

	Point newNodePoint = Point(newNode->point[0], newNode->point[1]); 
	Point parentPoint = Point(newNode->par_cost.load()->parent->point[0], newNode->par_cost.load()->parent->point[1]);

	if(checkCollision(parentPoint, newNodePoint, Point(position.x, position.y), RADIUS)) {
		pathFound = 1 ; 
		goalIndex = newNode->index;
		cout << "tree: Reached!! With a distance of " << newNode->par_cost.load()->cost << " units. " << endl << endl ;
		return true; 
	}
	return false;
}


/* Inserts nodes on the path from rootIndex till Point q such 
   that successive nodes on the path are not more than 
   JUMP_SIZE distance away */
void insertNodesInPath(int rootIndex, Point& q) {
	Point p = nodes[rootIndex] ; 
	if(!isEdgeObstacleFree(p, q)) return ;
	while(!(p == q)) {
		Point nxt = p.steer(q, JUMP_SIZE); 
		nodes.push_back(nxt); 
		parent.push_back(rootIndex);
		cost.push_back(cost[rootIndex] + distance(p, nxt));
		rootIndex = nodeCnt++ ; 
		p = nxt ; 
	}
}

/*  Rewires the parents of the tree greedily starting from 
	the new node found in this iterationsation as the parent */
void rewire(std::vector<int> nearby, int lastInserted) {
	// int lastInserted = nodeCnt - 1 ; 

	// printf("old rewire %ld\n", nearby.size());
	for(auto nodeIndex: nearby) {

		if (nodeIndex == lastInserted) continue;
		if (!isEdgeObstacleFree(nodes[nodeIndex], nodes[lastInserted])) continue;


		int par = lastInserted, cur = nodeIndex;

		// Rewire parents as much as possible (greedily)
		// printf("%f\n", ((cost[par] + distance(nodes[par], nodes[cur])) - cost[cur]));

		while( ((cost[par] + distance(nodes[par], nodes[cur])) - cost[cur]) <= EPS) { 

			int oldParent = parent[cur] ;
			parent[cur] = par; cost[cur] = cost[par] + distance(nodes[par], nodes[cur]);

			// printf("rewire node %d's parent from %d to %d\n", nodeIndex, oldParent, par);

			par = cur, cur = oldParent; 

		}
	}
}

/*  Rewires the parents of the tree greedily starting from 
	the new node found in this iterationsation as the parent */
void rewire(Kdtree::KdTreeNodeVec nearby, Kdtree::kdtree_node *newNode) {

	// printf("new rewire %ld\n", nearby.size());

	for (auto kdnode: nearby) {
		if (kdnode->par_cost.load()->parent == nullptr) continue;
		if (newNode->index == kdnode-> index) continue;

		Kdtree::kdtree_node *par = newNode; 
		Point parPnt = Point(par->point[0], par->point[1]);

		Kdtree::kdtree_node *cur = kdnode; 
		Point curPnt = Point(cur->point[0], cur->point[1]);

		if (!isEdgeObstacleFree(curPnt, parPnt)) continue;

		// printf("%f\n",  ((par->par_cost.load()->cost + distance(parPnt, curPnt)) - cur->par_cost.load()->cost));

		while( ((par->par_cost.load()->cost + distance(parPnt, curPnt)) - cur->par_cost.load()->cost) <= EPS) {


			Kdtree::rewire_par_cost *old = cur->par_cost;

			Kdtree::rewire_par_cost *newValue = new Kdtree::rewire_par_cost(); 

			newValue->parent = par; 
			newValue->cost = par->par_cost.load()->cost + distance(parPnt, curPnt); 
			
			if ((cur->par_cost).compare_exchange_weak(old, newValue, std::memory_order_release, std::memory_order_relaxed)) {



				// printf("rewire node %d's parent from %d to %d\n", kdnode->index, old->parent->index, par->index);

				par = cur; 
				cur = old->parent;
				parPnt = Point(par->point[0], par->point[1]);
				curPnt = Point(cur->point[0], cur->point[1]);
				delete old;

				if (cur == nullptr) break;
			}


		}

	}
}


/*	Runs one iteration of RRT depending on user choice 
	At least one new node is added on the screen each iteration. */
void RRT(Kdtree::KdTree &kdtree) {
	Point newPoint, nearestPoint, nextPoint ; bool updated = false ; int cnt = 0 ; 
	// Point nearestPoint2;
	int nearestIndex = 0 ; double minCost = INF;

	vector < int > nearby2 ; 
	int localNodeCnt = nodeCnt;

	vector <double > jumps(localNodeCnt);
	std::chrono::steady_clock::time_point begin;
	std::chrono::steady_clock::time_point end; 
	
	// int tid = omp_get_thread_num();
	int index = nodeCnt + omp_get_thread_num();
	
	
	while(!updated) {

		
		newPoint = pickRandomPoint(); 
   		Kdtree::CoordPoint queryPoint({newPoint.x, newPoint.y});


		// Find nearest point to the newPoint such that the next node 
		// be added in graph in the (nearestPoint, newPoint) while being obstacle free

		nearestPoint = Point(kdtree.root->point[0], kdtree.root->point[1]);

		bool done = false;
		int num_neighbors = 1;


		begin = std::chrono::steady_clock::now();
		int jump_idx, best_jump;
		Kdtree::kdtree_node* nearest_neighbor; 



		while (!done && num_neighbors < localNodeCnt + 1) {
			Kdtree::KdTreeNodeVec res;
			kdtree.k_nearest_neighbors(queryPoint, num_neighbors, &res);

			jump_idx = 0;
			best_jump = 0;
			

			for (auto kdnode: res) {
				// Make smaller jumps sometimes to facilitate passing through narrow passages 
				jumps[jump_idx] = randomCoordinate(0.3, 1.0) * JUMP_SIZE ; 
				Point pnt(kdnode->point[0], kdnode->point[1]);
			
				// if (kdnode->par_cost.load()->parent != nullptr) {
				// 	Point parentPnt(kdnode->par_cost.load()->parent->point[0], kdnode->par_cost.load()->parent->point[1]);

				// 	if(pathFound and randomCoordinate(0.0, 1.0) < 0.25) // Recalculate cost once in a while 
				// 		kdnode->par_cost.load()->cost = kdnode->par_cost.load()->parent->par_cost.load()->cost +  distance(parentPnt, pnt);
				// }



				if((pnt.distance(newPoint) - nearestPoint.distance(newPoint)) <= EPS and isEdgeObstacleFree(pnt, pnt.steer(newPoint, jumps[jump_idx]))) {
					nearestPoint = pnt; 
					nearest_neighbor = kdnode;
					nearestIndex = nearest_neighbor->index ; 
					best_jump = jump_idx;
					done = true;
					// break;
				}


				jump_idx++; 
			}


			num_neighbors++;
		}

		// double golden_dist = (nearestPoint2.x - newPoint.x) * (nearestPoint2.x - newPoint.x) + (nearestPoint2.y - newPoint.y) * (nearestPoint2.y - newPoint.y);
		// double tree_dist = (nearestPoint.x - newPoint.x) * (nearestPoint.x - newPoint.x) + (nearestPoint.y - newPoint.y) * (nearestPoint.y - newPoint.y);
		// if (abs(golden_dist - tree_dist) > 0.0001) {
		// 	printf("golden: %f %f %f \n", nearestPoint2.x, nearestPoint2.y, golden_dist);
		// 	printf("%d, tree: %f %f %f \n", num_neighbors, nearestPoint.x, nearestPoint.y, tree_dist);
		// }

		
		

		end = std::chrono::steady_clock::now();

		nextPoint = stepNear(nearestPoint, newPoint, jumps[best_jump]);

		if(!isEdgeObstacleFree(nearestPoint, nextPoint)) continue ; 


		if( (whichRRT == 1) or (!pathFound and whichRRT == 3)) {
			// This is where we don't do any RRT* optimization part 
			updated = true ;
			double cost_node = nearest_neighbor->par_cost.load()->cost + distance(nearestPoint, nextPoint);
			Kdtree::kdtree_node *newNode = kdtree.insert(Kdtree::CoordPoint ({nextPoint.x, nextPoint.y}), index, cost_node, nearest_neighbor);

	

			nodes[index] = nextPoint;
			parent[index] = nearestIndex;
			cost[index] = cost[nearestIndex] + distance(nearestPoint, nextPoint);

			if(!pathFound) checkDestinationReached(newNode);

			continue ; 
		}


		double minCost; 
		Kdtree::KdTreeNodeVec nearby;
    	kdtree.range_nearest_neighbors(Kdtree::CoordPoint({nextPoint.x, nextPoint.y}), DISK_SIZE, &nearby);
		minCost = nearest_neighbor->par_cost.load()->cost + distance(nearestPoint, nextPoint);
		Kdtree::kdtree_node* optim_parent = nearest_neighbor; 


		for (auto kdnode:nearby) {
			nearby2.push_back(kdnode->index);

			if (kdnode->index == nearest_neighbor->index) continue;
			Point nodePnt(kdnode->point[0], kdnode->point[1]);

			if (!isEdgeObstacleFree(nodePnt, nextPoint)) continue;


			if (((kdnode->par_cost.load()->cost + distance(nodePnt, nextPoint)) - minCost) <= EPS) {
				minCost = kdnode->par_cost.load()->cost + distance(nodePnt, nextPoint); 
				optim_parent = kdnode;
			}
		}
	

		Kdtree::kdtree_node *newNode = kdtree.insert(Kdtree::CoordPoint ({nextPoint.x, nextPoint.y}), index, minCost, optim_parent);

	

		nodes[index] = nextPoint;
		parent[index] = optim_parent->index;
		cost[index] = minCost;
	


		updated = true ; 

		if(!pathFound) checkDestinationReached(newNode);

		// rewire(nearby2, index);
		// rewire(nearby, newNode);
	}
}

int main(int argc, char* argv[]) {

	Kdtree::CoordPoint min_point({0, 0});
	Kdtree::CoordPoint max_point({WIDTH, HEIGHT});
	size_t num_dim = 2;
	Kdtree::KdTree kdtree(min_point, max_point, num_dim);

	std::chrono::steady_clock::time_point begin;
	std::chrono::steady_clock::time_point end; 

	whichRRT = 1; 
	start.x = 0;
	start.y = 0;
	stop.x = 500;
	stop.y = 500;
	obstacle_cnt = 0; 
	getInput(); 
	prepareInput();
#if defined(ON_MAC)
	sf::Vector2u dimensions(WIDTH, HEIGHT);
    sf::RenderWindow window(sf::VideoMode(dimensions, 1), "Basic Anytime RRT");
#else
    sf::RenderWindow window(sf::VideoMode(WIDTH, HEIGHT), "Basic Anytime RRT");
#endif

	nodeCnt = 1; nodes.push_back(start); int iterations = 0 ; 
	parent.push_back(0); cost.push_back(0);

	int nodeCnt2 = 1;
	kdtree.insert(Kdtree::CoordPoint ({start.x, start.y}), 0, 0, nullptr);
    sf::Time delayTime = sf::milliseconds(5);

    cout << endl << "Starting node is in Pink and Destination node is in Blue" << endl << endl ; 

	vector < Point > nodes2; 
	vector < int > parent2; 
	vector < double > cost2;
    while (window.isOpen())
    {

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

		nodes.resize(nodes.size() + num_threads);
		parent.resize(nodes.size() + num_threads);
		cost.resize(nodes.size() + num_threads);


		#pragma omp parallel for
		for (int i = 0; i < num_threads; i++) {
			// int tid = omp_get_thread_num();
			// if (tid==0)
				// begin = std::chrono::steady_clock::now();
	
			RRT(kdtree);
	

	
			
			// if (tid == 0)
				// end = std::chrono::steady_clock::now();

			// if (tid == 0)
			// 	std::cout << "total RRT = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;

		}
		iterations+=num_threads;
		nodeCnt += num_threads;

		if(iterations % 500 == 0) {
			cout << "Iterations: " << iterations << endl ; 
			if(!pathFound) cout << "Not reached yet :( " << endl ;
			else cout << "Shortest distance till now: " << cost[goalIndex] << " units." << endl ;
			cout << endl ;
		}

		sf::sleep(delayTime);
		window.clear();
		draw(window); 
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
