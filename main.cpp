#include <cstdio>
#include <iostream>
#include <fstream>
#include <vector>
#include <queue>

using namespace std;


char initialMap[25][25]; // map is used by 0 to (n-1)
int N, M; // N: number of rows, M: number of columns
int used[25][25][3]; // is user visited [x][y][orientaion] cell with orientaion; orientaion structure: [0] = single, [1] = vertical, [2] = horizontal
int startX, startY;
int goalX, goalY;
vector<char> road;
int numberOfExpandedNodes;
int globalCost;
int globalMaxDepth;
int globalDepth;

// reads from file
void readFile(const string& filename){ 
    string line;
    ifstream source;

    source.open(filename);
    source >> M;
    source >> N;
    getline(source, line);

    for(int i=0 ; i<N ; i++){
        getline(source, line);
        for(int j=0 ; j<line.length() ; j++){
            initialMap[i][j] = line[j];
            if (initialMap[i][j] == 's'){
                startX = i;
                startY = j;
            }
            else if (initialMap[i][j] == 'g'){
                goalX = i;
                goalY = j;
            }
        }
    }

    source.close();
}

// these structs are used for queues
struct node{
    node() : heuristcCost(0), depth(0){}
    int x,y;
    int orientation, cost;
    int fromX, fromY, fromOrientation, fromDirection;
    int heuristcCost;
    int depth;
};

struct greedyNode{
    greedyNode() : heuristcCost(0), depth(0){}
    int x,y;
    int orientation, cost;
    int fromX, fromY, fromOrientation, fromDirection;
    int heuristcCost;
    int depth;
};

// this operator is coded to get min heap. in c++ heap(priority queue) structure is max heap at default
bool operator<(const greedyNode& a,const greedyNode& b){
    return a.heuristcCost > b.heuristcCost;
};

// this operator is coded to get min heap. in c++ heap(priority queue) structure is max heap at default
bool operator<(const node& a,const node& b){
    return a.cost + a.heuristcCost > b.cost + b.heuristcCost;
};

// this is the heuristic method. I suggested that manhattan distance can be used for heuristic cost
int h(int x, int y){ 
    return abs(x - goalX) + abs(y - goalY);
}

// this hash function used for tracing the path that is discovered bfs like algorithms. return value of this function is assigned to used
int hashFunction(int a, int b, int c, int d){ // a: fromOrientation, b: fromY, c: fromX, d: fromDirection
    return a + 10*b + 1000*c + 100000*d;
}

// this method returns the x' and y' coordinates after left move from x and y coordinates with orientation 
pair<int, int> isLeftMoveAllowed(int x, int y, int orientation){
    int nextX = x;
    int nextY = y-1;

    if (nextY < 0){
        return make_pair(-1, -1);
    }
    if (orientation == 0){
        if (nextY-1 >= 0 && initialMap[nextX][nextY] != ' ' && initialMap[nextX][nextY-1] != ' ' && used[x][y-1][2] == -1){
            return make_pair(x, y-1);
        }
    }

    else if (orientation == 1){
        if (initialMap[nextX+1][nextY] != ' ' && initialMap[nextX][nextY] != ' ' && used[nextX][nextY][1] == -1 && nextX+1 < N){
            return make_pair(nextX, nextY);
        }
    }
    
    else if (orientation == 2){
        if (nextY-1 >= 0 && initialMap[nextX][nextY-1] != ' ' && used[nextX][nextY-1][0] == -1){
            return make_pair(nextX, nextY-1);
        }
    }

    return make_pair(-1, -1);
}

// this method returns the x' and y' coordinates after right move from x and y coordinates with orientation 
pair<int, int> isRightMoveAllowed(int x, int y, int orientation){
    int nextX = x;
    int nextY = y+1;

    if (nextY >= M){
        return make_pair(-1, -1);
    }
    if (orientation == 0){
        if (nextY+1 < M && initialMap[nextX][nextY] != ' ' && initialMap[nextX][nextY+1] != ' ' && used[nextX][nextY+1][2] == -1){
            return make_pair(nextX, nextY+1);
        }
    }

    else if (orientation == 1){
        if (nextX+1 < N && initialMap[nextX+1][nextY] != ' ' && initialMap[nextX][nextY] != ' ' && used[nextX][nextY][1] == -1){
            return make_pair(nextX, nextY);
        }
    }
    
    else if (orientation == 2){
        if (initialMap[nextX][nextY] != ' ' && used[nextX][nextY][0] == -1){
            return make_pair(nextX, nextY);
        }
    }

    return make_pair(-1, -1);
}

// this method returns the x' and y' coordinates after down move from x and y coordinates with orientation 
pair<int, int> isDownMoveAllowed(int x, int y, int orientation){
    int nextX = x+1;
    int nextY = y;

    if (nextX >= N){
        return make_pair(-1, -1);
    }
    if (orientation == 0){
        if (nextX+1 < N && initialMap[nextX][nextY] != ' ' && initialMap[nextX+1][nextY] != ' ' && used[x+1][y][1] == -1){
            return make_pair(x+1, y);
        }
    }

    else if (orientation == 1){
        if (nextX+1 < N && initialMap[nextX+1][nextY] != ' ' && used[nextX+1][nextY][0] == -1){
            return make_pair(nextX+1, nextY);
        }
    }
    
    else if (orientation == 2){
        if (nextY-1 >= 0 && initialMap[nextX][nextY] != ' ' && initialMap[nextX][nextY-1] != ' ' && used[nextX][nextY][2] == -1){
            return make_pair(nextX, nextY);
        }
    }
    return make_pair(-1, -1);
}

// this method returns the x' and y' coordinates after up move from x and y coordinates with orientation 
pair<int, int> isUpMoveAllowed(int x, int y, int orientation){
    int nextX = x-1;
    int nextY = y;
    
    if (nextX < 0){
        return make_pair(-1, -1);
    }

    if (orientation == 0){
        if (x-1 >= 0 && initialMap[nextX-1][nextY] != ' ' && initialMap[nextX][nextY] != ' ' && used[nextX-1][nextY][1] == -1){
            return make_pair(nextX-1, nextY);
        }
    }

    else if (orientation == 1){
        if (initialMap[nextX][nextY] != ' ' && used[nextX][nextY][0] == -1){
            return make_pair(nextX, nextY);
        }
    }
    else if (orientation == 2){
        if (nextY-1 >= 0 && initialMap[nextX][nextY] != ' ' && initialMap[nextX][nextY-1] != ' ' && used[nextX][nextY][2] == -1){
            return make_pair(nextX, nextY);
        }
    }

    return make_pair(-1, -1);
}

// this method returns the new orientation according moving direction and old orientation 
int getNewOrientation(int oldOrientation, int direction){
    int newOrientation = -1;

    if (oldOrientation == 0 && (direction == 1 || direction == 3)){
        newOrientation = 1;
    }
    else if (oldOrientation == 0){
        newOrientation = 2;
    }
    else if (oldOrientation == 1 && (direction == 1 || direction == 3)){
        newOrientation = 0;
    }
    else if (oldOrientation == 1){
        newOrientation = 1;
    }
    else if (oldOrientation == 2 && (direction == 1 || direction == 3)){
        newOrientation = 2;
    }
    else if (oldOrientation == 2){
        newOrientation = 0;
    }
    return newOrientation;
}

// this method selects the corresponding move
pair<int, int> isMoveAllowed(int x, int y, int direction, int orientation){
    switch (direction){
        case 1:
        return isUpMoveAllowed(x,y,orientation);
        case 2:
        return isRightMoveAllowed(x,y,orientation);
        case 3:
        return isDownMoveAllowed(x,y,orientation);
        case 0:
        return isLeftMoveAllowed(x,y,orientation);
        default:
        break; 
    }
    return make_pair(-1,-1);
}

// this method return char value for direction,  0:left, 1:up, 2: right, 3: down
char getCharForDirection(int k){
    switch (k){
        case 0:
        return 'L';
        case 1:
        return 'U';
        case 2:
        return 'R';
        case 3:
        return 'D';
        default:
        return '-';
    }
}

int dfs(int orientation, int x, int y, int cost, int depth){
    used[x][y][orientation] = 1;
    numberOfExpandedNodes++;

    if(depth > globalMaxDepth){
        globalMaxDepth = depth;
    }
    if (orientation == 0 && initialMap[x][y] == 'g'){
        globalCost = cost;
        globalDepth = depth;
        return 1;
    }

    for(int i=0 ; i<4 ; i++){
        int newOrientation = getNewOrientation(orientation, i);
       
        pair<int, int> moveAllowedTo = isMoveAllowed(x,y,i,orientation);
        if (moveAllowedTo.first != -1){
            int addCost = 1;
            if((orientation == 1 || orientation == 2) && newOrientation == 0){
                addCost += 2;
            }

            int solutionFound = dfs(newOrientation, moveAllowedTo.first, moveAllowedTo.second, cost + addCost, depth+1);
            if (solutionFound != 0){
                char ch = getCharForDirection(i);
                road.push_back(ch);
                return 1;
            }
        }
    }
    used[x][y][orientation] = -1;
    return 0;
}

// bfs algorithm: pushes starting state to queue and starts searching. there is no priority in this queue (fifo).
// I used hash function to back trace from goal state to starting state.
void bfs(){
    queue <node> q;

    node a;
    a.x = startX;
    a.y = startY;
    a.cost = 0;
    a.orientation = 0;
    a.fromX = startX;
    a.fromY = startY;
    a.fromOrientation = 0;
    a.depth = 0;

    q.push(a);

    while(q.size() > 0){
        node front = q.front();
        q.pop();

        if (front.depth > globalMaxDepth){
            globalMaxDepth = front.depth;
        }

        if (used[front.x][front.y][front.orientation] == -1){
            used[front.x][front.y][front.orientation] = hashFunction(front.fromOrientation, front.fromY, front.fromX, front.fromDirection);
            numberOfExpandedNodes++;
            for(int i=0 ; i<4 ; i++){
                int newOrientation = getNewOrientation(front.orientation, i);
               
                pair<int, int> moveAllowedTo = isMoveAllowed(front.x,front.y,i,front.orientation);
                if (moveAllowedTo.first != -1){
                    node newNode;
                    newNode.x = moveAllowedTo.first;
                    newNode.y = moveAllowedTo.second;
                    newNode.orientation = newOrientation;
                    newNode.cost = front.cost + 1;
                    newNode.fromX = front.x;
                    newNode.fromY = front.y;
                    newNode.fromOrientation = front.orientation;
                    newNode.fromDirection = i;
                    newNode.depth = front.depth + 1;

                    if((front.orientation == 1 || front.orientation == 2) && newOrientation == 0){
                        newNode.cost += 2;
                    }

                    if (initialMap[newNode.x][newNode.y] == 'g' && newNode.orientation == 0){
                        if (newNode.depth > globalMaxDepth){
                            globalMaxDepth = newNode.depth;
                        }
                        globalCost = newNode.cost;
                        used[newNode.x][newNode.y][newNode.orientation] = hashFunction(newNode.fromOrientation, newNode.fromY, newNode.fromX, newNode.fromDirection);
                        return;
                    }
                    q.push(newNode);
                }
            }    
        }
    }
}

// ucs algorithm: pushes starting state to priority queue and starts searching. priority in this queue is the cost.
// I used hash function to back trace from goal state to starting state.
void ucs(){
    priority_queue <node> q;

    node a;
    a.x = startX;
    a.y = startY;
    a.cost = 0;
    a.orientation = 0;
    a.fromX = startX;
    a.fromY = startY;
    a.fromOrientation = 0;
    q.push(a);

    while(!q.empty()){
        node front = q.top();
        q.pop();
        numberOfExpandedNodes++;
        if (front.depth > globalMaxDepth){
            globalMaxDepth = front.depth;
        }

        if (used[front.x][front.y][front.orientation] == -1){
            used[front.x][front.y][front.orientation] = hashFunction(front.fromOrientation, front.fromY, front.fromX, front.fromDirection);
            for(int i=0 ; i<4 ; i++){
                int newOrientation = getNewOrientation(front.orientation, i);
               
                pair<int, int> moveAllowedTo = isMoveAllowed(front.x,front.y,i,front.orientation);
                if (moveAllowedTo.first != -1){
                    node newNode;
                    newNode.x = moveAllowedTo.first;
                    newNode.y = moveAllowedTo.second;
                    newNode.orientation = newOrientation;
                    newNode.cost = front.cost + 1;
                    newNode.fromX = front.x;
                    newNode.fromY = front.y;
                    newNode.fromOrientation = front.orientation;
                    newNode.fromDirection = i;
                    newNode.depth = front.depth + 1;

                    if((front.orientation == 1 || front.orientation == 2) && newOrientation == 0){
                        newNode.cost += 2;
                    }
                     if (initialMap[newNode.x][newNode.y] == 'g' && newNode.orientation == 0){
                        if (newNode.depth > globalMaxDepth){
                            globalMaxDepth = newNode.depth;
                        }
                        globalCost = newNode.cost;
                        used[newNode.x][newNode.y][newNode.orientation] = hashFunction(newNode.fromOrientation, newNode.fromY, newNode.fromX, newNode.fromDirection);
                        return;
                    }
                    q.push(newNode);
                }
            }    
        }
    }
}

// bfs algorithm: pushes starting state to queue and starts searching. priority in this queue is the cost + heuristic cost
// I used hash function to back trace from goal state to starting state.
void aStar(){
    priority_queue <node> q;

    node a;
    a.x = startX;
    a.y = startY;
    a.cost = 0;
    a.orientation = 0;
    a.fromX = startX;
    a.fromY = startY;
    a.fromOrientation = 0;
    q.push(a);

    while(!q.empty()){
        node front = q.top();
        q.pop();
        if (front.depth > globalMaxDepth){
            globalMaxDepth = front.depth;
        }

        if (used[front.x][front.y][front.orientation] == -1){
            numberOfExpandedNodes++;
            used[front.x][front.y][front.orientation] = hashFunction(front.fromOrientation, front.fromY, front.fromX, front.fromDirection);
            for(int i=0 ; i<4 ; i++){
                int newOrientation = getNewOrientation(front.orientation, i);
               
                pair<int, int> moveAllowedTo = isMoveAllowed(front.x,front.y,i,front.orientation);
                if (moveAllowedTo.first != -1){
                    node newNode;
                    newNode.x = moveAllowedTo.first;
                    newNode.y = moveAllowedTo.second;
                    newNode.orientation = newOrientation;
                    newNode.cost = front.cost + 1;
                    newNode.fromX = front.x;
                    newNode.fromY = front.y;
                    newNode.fromOrientation = front.orientation;
                    newNode.heuristcCost = h(newNode.x, newNode.y);
                    newNode.fromDirection = i;
                    newNode.depth = front.depth + 1;

                    if((front.orientation == 1 || front.orientation == 2) && newOrientation == 0){
                        newNode.cost += 2;
                    }
                     if (initialMap[newNode.x][newNode.y] == 'g' && newNode.orientation == 0){
                        if (newNode.depth > globalMaxDepth){
                            globalMaxDepth = newNode.depth;
                        }
                        globalCost = newNode.cost;
                        used[newNode.x][newNode.y][newNode.orientation] = hashFunction(newNode.fromOrientation,
                                                                                       newNode.fromY, newNode.fromX,
                                                                                       newNode.fromDirection);
                        return;
                    }
                    q.push(newNode);
                }
            }    
        }
    }
}

// bfs algorithm: pushes starting state to queue and starts searching. priority in this queue is the heuristic cost
// I used hash function to back trace from goal state to starting state.
void greedy(){
    priority_queue <greedyNode> q;

    greedyNode a;
    a.x = startX;
    a.y = startY;
    a.cost = 0;
    a.orientation = 0;
    a.fromX = startX;
    a.fromY = startY;
    a.fromOrientation = 0;
    q.push(a);

    while(!q.empty()){
        greedyNode front = q.top();
        q.pop();
        if (front.depth > globalMaxDepth){
            globalMaxDepth = front.depth;
        }

        if (used[front.x][front.y][front.orientation] == -1){
            numberOfExpandedNodes++;
            used[front.x][front.y][front.orientation] = hashFunction(front.fromOrientation, front.fromY, front.fromX, front.fromDirection);
            for(int i=0 ; i<4 ; i++){
                int newOrientation = getNewOrientation(front.orientation, i);
               
                pair<int, int> moveAllowedTo = isMoveAllowed(front.x,front.y,i,front.orientation);
                if (moveAllowedTo.first != -1){
                    greedyNode newNode;
                    newNode.x = moveAllowedTo.first;
                    newNode.y = moveAllowedTo.second;
                    newNode.orientation = newOrientation;
                    newNode.cost = front.cost + 1;
                    newNode.fromX = front.x;
                    newNode.fromY = front.y;
                    newNode.fromOrientation = front.orientation;
                    newNode.heuristcCost = h(newNode.x, newNode.y);
                    newNode.fromDirection = i;
                    newNode.depth = front.depth + 1;

                    if((front.orientation == 1 || front.orientation == 2) && newOrientation == 0){
                        newNode.cost += 2;
                    }
                     if (initialMap[newNode.x][newNode.y] == 'g' && newNode.orientation == 0){
                        if (newNode.depth > globalMaxDepth){
                            globalMaxDepth = newNode.depth;
                        }
                        globalCost = newNode.cost;
                        used[newNode.x][newNode.y][newNode.orientation] = hashFunction(newNode.fromOrientation, newNode.fromY, newNode.fromX, newNode.fromDirection);
                        return;
                    }
                    q.push(newNode);
                }
            }    
        }
    }
}

void generateRoad(){
    int a = goalX, b = goalY, c = 0, direction;
    
    while(1){
        if (a == startX && b == startY){
            break;
        }
        int hashedValue = used[a][b][c];

        c = (hashedValue%10);
        hashedValue /= 10;
        b = hashedValue % 20;
        hashedValue /= 100;
        a = hashedValue % 20;
        hashedValue /= 100;
                
        char ch = getCharForDirection(hashedValue);
        road.push_back(ch);
    }
}

int main(int argc,  char **argv){
	string charactersFilename(argv[1]);
    string functionName(argv[2]);
	for(int i=0 ; i<20 ; i++)
		for(int j=0 ; j<20 ; j++){
			initialMap[i][j] = ' ';
		}

	readFile(charactersFilename);
    memset(used,-1,sizeof used);
    
    if (functionName == "dfs"){
        int asd = dfs(0, startX, startY, 0, 0);
    }
    else{
        if (functionName == "as"){
            aStar();
        }
        else if (functionName == "bfs"){
            bfs();
        }
        else if (functionName == "ucs"){
            ucs();
        }
        else if (functionName == "gs"){
            greedy();
        }
        generateRoad();
        globalDepth = road.size();
        numberOfExpandedNodes = 0;
        
        for(int i=0 ; i<25 ; i++){
            for(int j=0 ; j<25 ; j++){
                for(int k=0 ; k<3 ; k++){
                    if (used[i][j][k] != -1){
                        numberOfExpandedNodes++;
                    }
                }
            }

        }
    }
    
    
    cout << globalCost << " " << numberOfExpandedNodes-1 << " " << globalMaxDepth << " " << globalDepth << endl;

    for(int i = road.size()-1 ; i>=0 ; i--){
        cout << road[i];
    }

    cout << endl;

	return 0;
}

