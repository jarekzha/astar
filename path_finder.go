package astar

import (
	"math"

	"github.com/pkg/errors"
)

var openList = Heap{}
var startLoc int = 0
var endLoc int = 0
var grid *Grid
var locToClosed = make(map[int]bool)
var locToOpen = make(map[int]bool)
var locToG = make(map[int]float64)
var locToH = make(map[int]float64)
var locToF = make(map[int]int)
var locToParent = make(map[int]int)

var SQRT2 = math.Sqrt2

//Manhattan distance.
func heuristic(dx, dy float64) float64 {
	return math.Sqrt(dx*dx + dy*dy)
}

// backtrace according to the parent records and return the path.
// (including both start and end nodes)
// int node End node
// returns the path array
func backtrace(node int, smooth bool) [][]int {
	path := [][]int{}
	var lastVecx, lastVecy int

	// append end node
	coords := []int{node >> 16, node & 0xffff}
	path = append(path, coords)

	// backtrace and ignore collinear node
	for locToParent[node] != 0 {
		node = locToParent[node]
		coords = []int{node >> 16, node & 0xffff}

		replace := false
		if smooth {
			vecx := coords[0] - path[0][0]
			vecy := coords[1] - path[0][1]
			if lastVecx != 0 || lastVecy != 0 {
				if cross(vecx, vecy, lastVecx, lastVecy) == 0 {
					// fmt.Printf("check corrd(%d,%d) vec(%d,%d) lastVec(%d,%d) replace\n",
					// 	coords[0], coords[1],
					// 	vecX, vecY, lastVecX, lastVecY)
					replace = true
				}
			}

			lastVecx = vecx
			lastVecy = vecy
		}

		// append node
		if !replace {
			path = append(path, []int{0})
			copy(path[1:], path)
		}
		path[0] = coords
	}

	// trim corner
	if smooth {
		path = smoothPath(path)
	}

	return path
}

// smooth path by triming corner
func smoothPath(path [][]int) [][]int {
	if len(path) < 2 {
		return path
	}

	for i := 1; i < len(path)-1; i++ {
		pre := path[i-1]
		next := path[i+1]
		if grid.IsLineWalkable(pre[0], pre[1], next[0], next[1]) {
			path = append(path[:i], path[i+1:]...)
			i--
		}
	}

	return path
}

func FindPathByBrickLoc(theGrid *Grid, start int, end int) ([][]int, error) {
	return FindPath(grid,
		start>>16, start&0xffff, end>>16, end&0xffff,
		false, false, false)
}

// find a path of giving x, y brick locations
func FindPath(theGrid *Grid,
	startX, startY, endX, endY int,
	allowDiagonal, crossCorners, smooth bool) ([][]int, error) {

	//validate args
	if startX < 0 || startY < 0 || endX < 0 || endY < 0 {
		return nil, errors.New("x and y positions cannot be less then 0")
	}

	startLoc = startX<<16 | startY
	endLoc = endX<<16 | endY
	grid = theGrid

	// set the `g` and `f` value of the start node to be 0
	locToG[startLoc] = 0
	locToF[startLoc] = 0

	openList.Reset(locToF)

	openList.Push(startLoc)
	locToOpen[startLoc] = true

	for openList.IsNotEmpty() {
		//pop the position of node which has the minimum `f` value.
		node := openList.Pop()

		locToClosed[node] = true

		if node == endLoc {
			//	fmt.Println("[syncfinder_astar::findPath] hit end brick")
			coordPath := backtrace(node, smooth)
			return coordPath, nil
		}

		//get neighbors of the current node
		nodeX := node >> 16
		nodeY := node & 0xffff
		neighbors := grid.GetNeighbors(nodeX, nodeY, allowDiagonal, crossCorners)

		//		fmt.Println("[syncfinder_astar::findPath] process node:#{node}, x:#{nodeX}, y:#{nodeY},neighbors:#{neighbors}")

		for _, neighbor := range neighbors {
			if locToClosed[neighbor] == true {
				// fmt.Println("[syncfinder_astar::findPath] met closed node@#{neighbor}, x:#{neighbor >>> 16}, y:#{neighbor&0xffff}")
				continue
			}

			x := neighbor >> 16
			y := neighbor & 0xffff

			// get the distance between current node and the neighbor
			// and calculate the next g score
			ng := locToG[node]

			if x == nodeX || y == nodeY {
				ng = ng + 1
			} else {
				ng = ng + SQRT2
			}

			// check if the neighbor has not been inspected yet, or
			// can be reached with smaller cost from the current node
			if locToOpen[neighbor] == false || (ng < locToG[neighbor]) {
				locToG[neighbor] = ng

				if locToH[neighbor] == 0 {
					locToH[neighbor] = heuristic(math.Abs(float64(x-endX)), math.Abs(float64(y-endY)))
				}

				locToF[neighbor] = int(locToG[neighbor] + locToH[neighbor])
				neighborNode := x<<16 | y
				locToParent[neighborNode] = node

				if locToOpen[neighbor] == false {
					openList.Push(neighborNode)
					locToOpen[neighbor] = true
				} else {
					// the neighbor can be reached with smaller cost.
					// Since its f value has been updated, we have to
					// update its position in the open list
					openList.UpdateItem(neighborNode)
				}
			}
		}
	}
	// fail to find the path
	return nil, errors.New("failed to find the path")
}
