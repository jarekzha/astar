package astar

import (
	"fmt"
	"math"
	"math/rand"
	"strconv"
	"strings"

	"github.com/pkg/errors"
)

// a data representation of the map grid, on which path finding occurs
type Grid struct {
	Width  int
	Height int
	Bytes  []byte
}

// generate a map buffer from the given 2d array
// int width of the map
// int height of the map
// [][]int 2D map array,  1: means blocked, 0: means walkable
func BytesFrom2DArray(width int, height int, array2d [][]int) ([]byte, error) {
	if width <= 0 || height <= 0 {
		return nil, errors.New("width or height is 0 or less")
	}

	var bufW float64 = float64(width)
	var bufH float64 = float64(height)
	lenOfBuf := math.Ceil(bufW * bufH / 8)

	buf := make([]byte, int(lenOfBuf))
	for i := range buf { // fill all bits as blocked
		buf[i] = 255
	}

	for y := 0; y < len(array2d); y++ {
		row := array2d[y]
		for x := 0; x < len(row); x++ {
			col := row[x]

			if col == 0 { // 0 means walkable
				index := int(y)*width + int(x)
				byteIndex := index >> 3
				offset := 7 - (index % 8) // write from high to low
				bytef := buf[byteIndex]
				bytef = bytef ^ 1<<offset
				buf[byteIndex] = bytef
				// fmt.Printf("[grid$::bytesFrom2DArray] walkable at x:%v, y:%v, row:%v, index:%v, offset:%v, byteIndex:%v", x, y, row, index, offset, byteIndex)
			}
		}
	}
	return buf, nil
}

func (grid *Grid) checkValues() error {
	if grid.Width < 0 || grid.Height < 0 {
		return errors.New("bad arguments for width or height. they should be greater than 0")
	}

	if len(grid.Bytes) != int(math.Ceil(float64(grid.Width)*float64(grid.Height))/8) {
		return errors.New("bytes length mismatch")
	}

	return nil
}

// Determine whether the node at the given position is walkable.
// (Also returns false if the position is outside the grid.)
// int x - The x coordinate of the node.
// int y - The y coordinate of the node.
func (grid *Grid) IsWalkableAt(x int, y int) bool {
	if x < 0 || y < 0 || x >= grid.Width || y >= grid.Height { //out of bounds
		return false
	}
	index := (y*grid.Width + x)
	bytePos := index >> 3
	offset := 7 - index%8
	bytef := grid.Bytes[bytePos]
	// NOTE:
	// 1: means blocked, 0: means walkable
	return (bytef >> offset & 1) == 0

}

// Check line from a to b is walkable
// (Also returns false if the position is outside the grid.)
func (grid *Grid) IsLineWalkable(ax, ay, bx, by int) bool {
	if !grid.IsWalkableAt(ax, ay) || !grid.IsWalkableAt(bx, by) {
		return false
	}

	var k float64
	var kn int
	if ax == bx {
		kn = -1
		k = 0
	} else {
		kn = abs((by - ay) / (bx - ax))
		k = float64(by-ay) / float64(bx-ax)
	}

	// traverse the line
	if kn == 0 {
		step := sign(bx - ax)
		for x := ax; x != bx; x += step {
			y := float64(ay) + float64(x-ax)*k
			// fmt.Printf("check line(%d,%d) from a(%d,%d), b(%d,%d)\n", x, y, ax, ay, bx, by)
			if !grid.IsWalkableAt(x, int(math.Ceil(y))) {
				return false
			}
			if !grid.IsWalkableAt(x, int(math.Floor(y))) {
				return false
			}
		}
	} else {
		step := sign(by - ay)
		for y := ay; y != by; y += step {
			if kn == -1 {
				if !grid.IsWalkableAt(ax, y) {
					return false
				}
			} else {
				x := float64(ax) + float64(y-ay)/k
				if !grid.IsWalkableAt(int(math.Ceil(x)), y) {
					return false
				}
				if !grid.IsWalkableAt(int(math.Floor(x)), y) {
					return false
				}
			}
		}
	}

	return true
}

// Set whether the node on the given position is walkable.
// NOTE: throws exception if the coordinate is not inside the grid.
// int x - The x coordinate of the node.
// int y - The y coordinate of the node.
func (grid *Grid) SetWalkableAt(x int, y int, walkable bool) error {
	if x < 0 || y < 0 || x >= grid.Width || y >= grid.Height { //out of bounds
		return errors.New("Out of bounds")
	}

	index := (y*grid.Width + x)
	bytePos := index >> 3
	offset := 7 - index%8
	bytef := grid.Bytes[bytePos]

	if !grid.IsWalkableAt(x, y) {
		grid.Bytes[bytePos] = bytef ^ 1<<offset
	}
	return nil
}

// returns []int each uint present x(high 16 bit) and y(low 16 bit)
// Get the neighbors of the given node.
//
//     offsets      diagonalOffsets:
//   +---+---+---+    +---+---+---+
//   |   | 0 |   |    | 0 |   | 1 |
//   +---+---+---+    +---+---+---+
//   | 3 |   | 1 |    |   |   |   |
//   +---+---+---+    +---+---+---+
//   |   | 2 |   |    | 3 |   | 2 |
//   +---+---+---+    +---+---+---+
//
// When allowDiagonal is true, if offsets[i] is valid, then
// diagonalOffsets[i] and
// diagonalOffsets[(i + 1) % 4] is valid.
// int  x
// int y
// bool allowDiagonal
// bool crossCorners
// returns []int a list of walkable neighbors brick loc
func (grid *Grid) GetNeighbors(x int, y int, allowDiagonal bool, crossCorners bool) []int {

	if x < 0 || y < 0 || x >= grid.Width || y >= grid.Height { //out of bounds
		return nil
	}
	// TODO:
	//   should return null when brick is blocked?

	neighbors := []int{}
	var s0 bool
	var s1 bool
	var s2 bool
	var s3 bool

	var d0 bool
	var d1 bool
	var d2 bool
	var d3 bool

	// ↑
	if grid.IsWalkableAt(x, y-1) {
		neighbors = append(neighbors, x<<16|(y-1))
		s0 = true
	}

	// →
	if grid.IsWalkableAt(x+1, y) {
		neighbors = append(neighbors, (x+1)<<16|y)
		s1 = true
	}
	// ↓
	if grid.IsWalkableAt(x, y+1) {
		neighbors = append(neighbors, x<<16|(y+1))
		s2 = true
	}

	// ←
	if grid.IsWalkableAt(x-1, y) {
		neighbors = append(neighbors, (x-1)<<16|y)
		s3 = true
	}

	if allowDiagonal == false {
		return neighbors
	}

	if crossCorners {
		d0 = s3 || s0
		d1 = s0 || s1
		d2 = s1 || s2
		d3 = s2 || d3
	} else {
		d0 = s3 && s0
		d1 = s0 && s1
		d2 = s1 && s2
		d3 = s2 && s3
	}

	// ↖
	if d0 && grid.IsWalkableAt(x-1, y-1) {
		neighbors = append(neighbors, (x-1)<<16|(y-1))
	}

	// ↗
	if d1 && grid.IsWalkableAt(x+1, y-1) {
		neighbors = append(neighbors, (x+1)<<16|(y-1))
	}

	// ↘
	if d2 && grid.IsWalkableAt(x+1, y+1) {
		neighbors = append(neighbors, (x+1)<<16|(y+1))
	}

	// ↙
	if d3 && grid.IsWalkableAt(x-1, y+1) {
		neighbors = append(neighbors, (x-1)<<16|(y+1))
	}
	return neighbors
}

// get a walkable brick location
func (grid *Grid) GetRandomWalkableBrick() int {
	for {
		x := int((rand.Float64() * float64(grid.Width))) >> 0
		y := int((rand.Float64() * float64(grid.Height))) >> 0
		if grid.IsWalkableAt(x, y) {
			return x<<16 | y
		}
	}
}

// print out the block data for human inspection
// @param  startLoc the start brick loc
// @param  endLoc the end brick loc
// @param  path array of point
// returns a string to describe this instance
func (grid *Grid) ToString(startLoc int, endLoc int, path [][]int) string {
	markpoints := make(map[int]string)

	for i, loc := range path {
		x := loc[0]
		y := loc[1]
		markpoints[x<<16|y] = strconv.Itoa(i % 10)
	}

	markpoints[startLoc] = "S"
	markpoints[endLoc] = "E"

	result := fmt.Sprintf("[Grid(width=%v, height=%v)]\nDump: ░=walkable, ▓=blocked", grid.Width, grid.Height)

	for y := 0; y < int(grid.Height); y++ {
		arr := []string{}
		for x := 0; x < int(grid.Width); x++ {
			if markpoints[int(x<<16|y)] != "" {
				arr = append(arr, markpoints[int(x<<16|y)])
			} else {
				if grid.IsWalkableAt(int(x), int(y)) {
					arr = append(arr, "░")
				} else {
					arr = append(arr, "▓")
				}
			}
		}
		result = result + ("\n" + strings.Join(arr[:], ""))
	}

	return result

}
