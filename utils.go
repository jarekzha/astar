/**
 * description: util functions
 * author: jarekzha@gmail.com
 * date: Jun 10, 2022
 */
package astar

// vector cross
func cross(ax, ay, bx, by int) int {
	return ax*by - ay*bx
}

// abs
func abs(x int) int {
	if x < 0 {
		return -x
	}
	return x
}

func sign(x int) int {
	if x < 0 {
		return -1
	}
	return 1
}
