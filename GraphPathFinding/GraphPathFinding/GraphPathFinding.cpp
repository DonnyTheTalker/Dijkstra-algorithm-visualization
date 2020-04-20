#include <vector>
#include <algorithm>
#include <queue>

#include "olcConsoleGameEngine.h"

// All right belong to @Javidx9 (OneLoneCoder.com)

using namespace std;

class GraphAlgorithms : public olcConsoleGameEngine
{

private:

	// simple node struct
	struct sNode
	{
		bool bObstacle = false;
		bool bVisited = false; 
		int nLocalGoal;  // exact distance from starting goal
		int x, y;
		vector <sNode*> vecNeighbours;
		sNode* parent;
	};

	int currectPathPos; // currect node position in array of all visited nodes

	sNode* nodes = nullptr;
	sNode* nodeStart = nullptr;
	sNode* nodeEnd = nullptr;

	void (GraphAlgorithms::* MainAlgo)(); // function pointer in case we want
										  // to add some new algorithms

	vector <sNode*> visitedNodes;

	int nMapWidth = 8;
	int nMapHeight = 8;

	// Path Finding Algorithms
private:

	void DijkstraFind()
	{

		for (int x = 0; x < nMapWidth; x++)
			for (int y = 0; y < nMapHeight; y++) {

				nodes[y * nMapWidth + x].parent = nullptr;
				nodes[y * nMapWidth + x].bVisited = false; 
				nodes[y * nMapWidth + x].nLocalGoal = int(1e9); // probably max distance possible

			}

		// heuristics for Dijkstra algorithm
		auto DijkstraCompare = [&](pair <sNode*, int> lhs, pair <sNode*, int> rhs)
		{

			return lhs.first->nLocalGoal > rhs.first->nLocalGoal;

		}; 

		priority_queue <pair <sNode*, int>, vector <pair <sNode*, int> >,
			decltype(DijkstraCompare)> pQ(DijkstraCompare); // way to use custom comparator in stl data-structures

		nodeStart->nLocalGoal = 0;

		pQ.push(make_pair(nodeStart, 0));

		while (!pQ.empty()) {

			sNode* curNode = pQ.top().first;

			if (!curNode->bVisited)
				visitedNodes.push_back(curNode);

			curNode->bVisited = true;
			int dist = pQ.top().second;

			pQ.pop();

			if (curNode->nLocalGoal > dist) // no possibility to do relaxation
				continue;

			if (curNode == nodeEnd) // end reached!
				break;

			for (auto nextNode : curNode->vecNeighbours) {

				if (!nextNode->bObstacle) {

					// producing relaxation (minimazing shortest path to node)
					if (nextNode->nLocalGoal > dist + 1) {

						nextNode->nLocalGoal = dist + 1;
						nextNode->parent = curNode;
						pQ.push(make_pair(nextNode, dist + 1));

					}

				}

			}

		}

	}


	// Drawing Path Functions
private:

	void DrawBoard()
	{
		int nNodeSize = 9;
		int nNodeBorder = 2;

		Fill(0, 0, ScreenWidth(), ScreenHeight(), L' ');

		for (int x = 0; x < nMapWidth; x++)
			for (int y = 0; y < nMapHeight; y++) {

				// drawing all the edges before nodes
				for (auto n : nodes[y * nMapWidth + x].vecNeighbours) {

					DrawLine(x * nNodeSize + nNodeSize / 2, y * nNodeSize + nNodeSize / 2,
						n->x * nNodeSize + nNodeSize / 2, n->y * nNodeSize + nNodeSize / 2, PIXEL_SOLID, FG_DARK_BLUE);

				}

			}


	}

	void DrawNodes()
	{

		int nNodeSize = 9;
		int nNodeBorder = 2;

		// white color -> obstacle
		// green color -> start node
		// red color -> end node
		// dark blue color -> unvisited node
		// light blue color -> visited node
		for (int x = 0; x < nMapWidth; x++)
			for (int y = 0; y < nMapHeight; y++) {

				if (nodes[y * nMapWidth + x].bObstacle)
					Fill(x * nNodeSize + nNodeBorder, y * nNodeSize + nNodeBorder,
						(x + 1) * nNodeSize - nNodeBorder, (y + 1) * nNodeSize - nNodeBorder,
						PIXEL_HALF, FG_WHITE);

				else if (nodes[y * nMapWidth + x].bVisited)
					Fill(x * nNodeSize + nNodeBorder, y * nNodeSize + nNodeBorder,
						(x + 1) * nNodeSize - nNodeBorder, (y + 1) * nNodeSize - nNodeBorder,
						PIXEL_SOLID, FG_BLUE);

				else
					Fill(x * nNodeSize + nNodeBorder, y * nNodeSize + nNodeBorder,
						(x + 1) * nNodeSize - nNodeBorder, (y + 1) * nNodeSize - nNodeBorder,
						PIXEL_HALF, FG_BLUE);

				if (nodeStart == &nodes[y * nMapWidth + x])
					Fill(x * nNodeSize + nNodeBorder, y * nNodeSize + nNodeBorder,
						(x + 1) * nNodeSize - nNodeBorder, (y + 1) * nNodeSize - nNodeBorder,
						PIXEL_SOLID, FG_GREEN);

				else if (nodeEnd == &nodes[y * nMapWidth + x])
					Fill(x * nNodeSize + nNodeBorder, y * nNodeSize + nNodeBorder,
						(x + 1) * nNodeSize - nNodeBorder, (y + 1) * nNodeSize - nNodeBorder,
						PIXEL_SOLID, FG_RED);

			}

	}

	void PathDraw(GraphAlgorithms::sNode* n = nullptr)
	{

		int nNodeSize = 9;
		int nNodeBorder = 2;

		if (n == nullptr)
			n = nodeEnd;

		sNode* anc = n;

		while (anc->parent != nullptr) {

			int x = anc->x, y = anc->y;
			int x1 = anc->parent->x, y1 = anc->parent->y;

			DrawLine(x * nNodeSize + nNodeSize / 2, y * nNodeSize + nNodeSize / 2,
				x1 * nNodeSize + nNodeSize / 2, y1 * nNodeSize + nNodeSize / 2, PIXEL_SOLID, FG_YELLOW);

			anc = anc->parent;

		}

	}

public:

	GraphAlgorithms()
	{
		m_sAppName = L"Lmk - obstacle, shift+lmk - start node, ctrl+lmk - end node";
	}

protected:
	virtual bool OnUserCreate() override
	{
		currectPathPos = 0;

		nodes = new sNode[nMapWidth * nMapHeight];
		nodeStart = &nodes[0];
		nodeEnd = &nodes[nMapWidth * nMapHeight - 1];

		MainAlgo = &GraphAlgorithms::DijkstraFind;

		for (int x = 0; x < nMapWidth; x++)
			for (int y = 0; y < nMapHeight; y++) {

				nodes[y * nMapWidth + x].x = x;
				nodes[y * nMapWidth + x].y = y;
				nodes[y * nMapWidth + x].bObstacle = false;
				nodes[y * nMapWidth + x].parent = nullptr;
				nodes[y * nMapWidth + x].bVisited = false;

			}

		for (int x = 0; x < nMapWidth; x++)
			for (int y = 0; y < nMapHeight; y++) {

				// vertical and horizontal edges
				if (y > 0)
					nodes[y * nMapWidth + x].vecNeighbours.push_back(&nodes[(y - 1) * nMapWidth + (x + 0)]);
				if (y < nMapHeight - 1)
					nodes[y * nMapWidth + x].vecNeighbours.push_back(&nodes[(y + 1) * nMapWidth + (x + 0)]);
				if (x > 0)
					nodes[y * nMapWidth + x].vecNeighbours.push_back(&nodes[(y + 0) * nMapWidth + (x - 1)]);
				if (x < nMapWidth - 1)
					nodes[y * nMapWidth + x].vecNeighbours.push_back(&nodes[(y + 0) * nMapWidth + (x + 1)]);

				// optional diogonal graph edges
				/*if (y > 0 && x > 0)
					nodes[y * nMapWidth + x].vecNeighbours.push_back(&nodes[(y - 1) * nMapWidth + (x - 1)]);
				if (y < nMapHeight - 1 && x > 0)
					nodes[y*nMapWidth + x].vecNeighbours.push_back(&nodes[(y + 1) * nMapWidth + (x - 1)]);
				if (y > 0 && x < nMapWidth - 1)
					nodes[y * nMapWidth + x].vecNeighbours.push_back(&nodes[(y - 1) * nMapWidth + (x + 1)]);
				if (y < nMapHeight - 1 && x < nMapWidth - 1)
					nodes[y * nMapWidth + x].vecNeighbours.push_back(&nodes[(y + 1) * nMapWidth + (x + 1)]);*/

			}

		(this->*MainAlgo)(); // running main path finding algorithm

		return true;

	}

	virtual bool OnUserUpdate(float fElapsedTime) override
	{

		DrawBoard();

		int nNodeSize = 9;
		int nNodeBorder = 2;

		int nSelectedNodeX = m_mousePosX / nNodeSize;
		int nSelectedNodeY = m_mousePosY / nNodeSize;

		// handling mouse input
		if (nodeStart != &nodes[nSelectedNodeY * nMapWidth + nSelectedNodeX] &&
			nodeEnd != &nodes[nSelectedNodeY * nMapWidth + nSelectedNodeX])

			if (m_mouse[0].bReleased) {

				if (nSelectedNodeX >= 0 && nSelectedNodeX < nMapWidth &&
					nSelectedNodeY >= 0 && nSelectedNodeY < nMapHeight) {

					if (m_keys[VK_SHIFT].bHeld) {
						nodeStart = &nodes[nSelectedNodeY * nMapWidth + nSelectedNodeX];
						nodeStart->bObstacle = false;
					} else if (m_keys[VK_CONTROL].bHeld) {
						nodeEnd = &nodes[nSelectedNodeY * nMapWidth + nSelectedNodeX];
						nodeEnd->bObstacle = false;
					} else
						nodes[nSelectedNodeY * nMapWidth + nSelectedNodeX].bObstacle =
						!nodes[nSelectedNodeY * nMapWidth + nSelectedNodeX].bObstacle;

					currectPathPos = 0;
					visitedNodes.clear();

					(this->*MainAlgo)();

				}

			}

		DrawNodes();

		if (currectPathPos == visitedNodes.size()) // check if we can achieve end node
			if (nodeEnd->bVisited)
				currectPathPos--;
			else
				return true;

		PathDraw(visitedNodes[currectPathPos++]);

		Sleep(20); // some extra delay so that user
				   // won't have epilepsy

		return true;

	}

};

int main()
{

	GraphAlgorithms graph;
	graph.ConstructConsole(160, 100, 8, 8);
	graph.Start();


	return 0;

}