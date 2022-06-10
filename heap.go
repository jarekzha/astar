package path

//utils
type Heap struct {
	nodes  []int
	locToF map[int]int
}

func (heap *Heap) IsNotEmpty() bool {
	return len(heap.nodes) > 0
}

func (heap *Heap) siftDown(startPos int, pos int) {
	newNode := heap.nodes[pos]

	for pos > startPos {
		parentpos := (pos - 1) >> 1
		parentNode := heap.nodes[parentpos]
		if heap.locToF[newNode] < heap.locToF[parentNode] {
			heap.nodes[pos] = parentNode
			pos = parentpos
			continue
		}
		break //TODO: need double check
	}
	heap.nodes[pos] = newNode
}

func (heap *Heap) siftUp(pos int) {
	endPos := len(heap.nodes)
	startPos := pos
	newNode := heap.nodes[pos]
	childPos := (pos << 1) + 1

	for childPos < endPos {
		rightPos := childPos + 1

		if rightPos < endPos && (heap.locToF[heap.nodes[childPos]] > heap.locToF[heap.nodes[rightPos]]) {
			childPos = rightPos
		}

		heap.nodes[pos] = heap.nodes[childPos]
		pos = childPos
		childPos = (pos << 1) + 1
	}

	heap.nodes[pos] = newNode
	heap.siftDown(startPos, pos)
}

//utils
func (heap *Heap) indexOf(element int) int {
	for k, v := range heap.nodes {
		if element == v {
			return k
		}
	}
	return -1 //not found.
}

func (heap *Heap) UpdateItem(node int) {
	pos := heap.indexOf(node)
	if pos < 0 {
		return
	}
	heap.siftDown(0, pos)
	heap.siftUp(pos)
}

// Push item onto heap, maintaining the heap invariant
func (heap *Heap) Push(node int) {
	heap.nodes = append(heap.nodes, node)
	heap.siftDown(0, len(heap.nodes)-1)
}

// Pop the smallest item off the heap, maintaining the heap invariant.
func (heap *Heap) Pop() int {
	lastelt := heap.nodes[len(heap.nodes)-1]
	heap.nodes = heap.nodes[:len(heap.nodes)-1]

	if len(heap.nodes) > 0 {
		returnitem := heap.nodes[0]
		heap.nodes[0] = lastelt
		heap.siftUp(0)
		return returnitem
	} else {
		return lastelt
	}
}

// reset the heap
func (heap *Heap) Reset(locToF map[int]int) {
	heap.nodes = heap.nodes[:0]
	heap.locToF = locToF
}
