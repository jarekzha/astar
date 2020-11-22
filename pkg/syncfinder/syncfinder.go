package syncfinder

import (
	g "github.com/ewol123/go-path/pkg/grid"
	 "github.com/ewol123/go-path/pkg/heap"
	"math"
	"github.com/pkg/errors"
)

var openList = heap.Heap{}
var startLoc int = 0
var endLoc int = 0
var grid = g.Grid{}
var locToClosed = make(map[int]bool)
var locToOpen = make(map[int]bool)
var locToG = make(map[int]float64)
var locToH = make(map[int]float64)
var locToF = make(map[int]int)
var locToParent = make(map[int]int)

var SQRT2 = math.Sqrt2

//Manhattan distance.
func Heuristic(dx float64, dy float64) float64 {
	return dx + dy
}

// Backtrace according to the parent records and return the path.
// (including both start and end nodes)
// int node End node
// returns the path array
func Backtrace(node int) ([]int, [][]int) {
	path := []int{}
	coordPath := [][]int{}

	path = append(path, node)
	for locToParent[node] != 0 {
		node = locToParent[node]
		path = append(path, 0)
		copy(path[1:], path)
		path[0] = node

		//append to coord path too
		coords := []int{node >> 16, node & 0xffff}
		coordPath = append(coordPath, []int{0})
		copy(coordPath[1:], coordPath)
		coordPath[0] = coords
	}
	return path, coordPath
}

type SyncFinder struct{}

func (syncFinder *SyncFinder) FindPathByBrickLoc(start int, end int, theGrid g.Grid) ([]int, [][]int, error) {
	return syncFinder.FindPath(start>>16, start&0xffff, end>>16, end&0xffff, theGrid, false, false)
}

// find a path of giving x, y brick locations
func (syncFinder *SyncFinder) FindPath(startX int, startY int, endX int, endY int, theGrid g.Grid, allowDiagonal bool, dontCrossCorners bool) ([]int, [][]int, error) {

	//validate args
	if startX < 0 || startY < 0 || endX < 0 || endY < 0 {
		return nil, nil, errors.New("x and y positions cannot be less then 0")
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
			path, coordPath := Backtrace(node)
			return path, coordPath, nil
		}

		//get neighbors of the current node
		nodeX := node >> 16
		nodeY := node & 0xffff
		neighbors := grid.GetNeighbors(nodeX, nodeY, allowDiagonal, dontCrossCorners)

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

				if locToH[neighbor] != 0 {
					locToH[neighbor] = locToH[neighbor]
				} else {
					locToH[neighbor] = Heuristic(math.Abs(float64(x-endX)), math.Abs(float64(y-endY)))
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
	return nil, nil, errors.New("failed to find the path")
}
