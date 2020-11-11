import time


class Pos(object):
    def __init__(self):
        self.x0 = None
        self.x1 = None
        self.y0 = None
        self.y1 = None
        self.path = []
        self.dist = 0

#   Overloaded == operator so that we can compare between objects.(BUT NOT THEIR PATH OR DIST VALUES)
    def __eq__(self, other):
        if self.x0 == other.x0 and self.y0 == other.y0 and self.x1 == other.x1 and self.y1 == other.y1:
            return True
        return False

    def __str__(self):
        print(self.x0, self.y0, self.x1, self.y1)

#   Hash function to store all of the coordinates in a
#   dictionary so we can access them faster.
    def __hash__(self):
        return hash((hash((self.x0, self.x1)), hash((self.y0, self.y1))))


def goal_pos(m):

    goal = Pos()
    for i in range(len(m)):
        for j in range(len(m[0])):
            if m[i][j] == 'G':
                    goal.x0 = i
                    goal.y0 = j
                    break

    return goal

# Compare the x and y values of the goal
# with our current position.


def goal_test(state, curr, goal):

    if state == 1:
        if curr.x0 is goal.y0 and curr.y0 is goal.y1:
            return True
    else:
        return False


#   Returns 3 if the block is horizontal upwards
#   Returns 2 if the block is horizontal sideways
#   Returns 1 if the block is vertical


def curr_state(position):

    if position.x1 is None:
        return 1
    elif position.y1 > position.y0:
        return 2
    elif position.x1 > position.x0:
        return 3

# After checking the current state of the block(vertical,horizontal sideways, horizontal upwards)
# the function possible moves returns all of the possible moves in an array. This function is our
# successor state function.


def possible_moves(pos, m):

    state = curr_state(pos)

    valid = ['O', 'G', 'S']
    moves = []

    if state is 1:
        if len(m[0]) - 1 >= pos.y0 + 2:
            if m[pos.x0][pos.y0 + 1] in valid and m[pos.x0][pos.y0 + 2] in valid:  # Check right
                #   Created different types of Pos variables for each possible successor
                #   to see what is pushed in debugger.
                type1 = Pos()
                type1.x0 = pos.x0
                type1.y0 = pos.y0 + 1
                type1.x1 = pos.x0
                type1.y1 = pos.y0 + 2
                moves.append(type1)

        if pos.y0 - 2 >= 0:
            if m[pos.x0][pos.y0 - 1] in valid and m[pos.x0][pos.y0 - 2] in valid:  # Check left
                type2 = Pos()
                type2.x0 = pos.x0
                type2.y0 = pos.y0 - 2
                type2.x1 = pos.x0
                type2.y1 = pos.y0 - 1
                moves.append(type2)
        if pos.x0 - 2 >= 0:
            if m[pos.x0 - 1][pos.y0] in valid and m[pos.x0 - 2][pos.y0] in valid:  # Check up
                type3 = Pos()
                type3.x0 = pos.x0 - 2
                type3.y0 = pos.y0
                type3.x1 = pos.x0 - 1
                type3.y1 = pos.y0
                moves.append(type3)
        if len(m) - 1 >= pos.x0 + 2:
            if m[pos.x0 + 1][pos.y0] in valid and m[pos.x0 + 2][pos.y0] in valid:  # Check down
                type4 = Pos()
                type4.x0 = pos.x0 + 1
                type4.y0 = pos.y0
                type4.x1 = pos.x0 + 2
                type4.y1 = pos.y0
                moves.append(type4)

    elif state is 2:
        if len(m[0]) - 1 >= pos.y1 + 1:
            if m[pos.x1][pos.y1 + 1] in valid:  # Check right
                type5 = Pos()
                type5.x0 = pos.x1
                type5.y0 = pos.y1 + 1
                type5.x1 = None
                type5.y1 = None
                moves.append(type5)
        if pos.y0 - 1 >= 0:
            if m[pos.x0][pos.y0 - 1] in valid:  # Check left
                type6 = Pos()
                type6.x0 = pos.x0
                type6.y0 = pos.y0 - 1
                type6.x1 = None
                type6.y1 = None
                moves.append(type6)
        if pos.x0 - 1 >= 0 and pos.x1 - 1 >= 0:
            if m[pos.x0 - 1][pos.y0] in valid and m[pos.x1 - 1][pos.y1] in valid:  # Check up
                type7 = Pos()
                type7.x0 = pos.x0 - 1
                type7.y0 = pos.y0
                type7.x1 = pos.x1 - 1
                type7.y1 = pos.y1
                moves.append(type7)
        if len(m) - 1 >= pos.x0 + 1 and len(m) - 1 >= pos.x1 + 1:
            if m[pos.x0 + 1][pos.y0] in valid and m[pos.x1 + 1][pos.y1] in valid:  # Check down
                type8 = Pos()
                type8.x0 = pos.x0 + 1
                type8.y0 = pos.y0
                type8.x1 = pos.x1 + 1
                type8.y1 = pos.y1
                moves.append(type8)

    elif state is 3:
        if len(m[0]) - 1 >= pos.y0 + 1 and len(m[0]) - 1 >= pos.y1 + 1:
            if m[pos.x0][pos.y0 + 1] in valid and m[pos.x1][pos.y1 + 1] in valid:  # Check right
                type9 = Pos()
                type9.x0 = pos.x0
                type9.y0 = pos.y0 + 1
                type9.x1 = pos.x1
                type9.y1 = pos.y1 + 1
                moves.append(type9)
        if pos.y0 - 1 >= 0 and pos.y1-1 >= 0:
            if m[pos.x0][pos.y0 - 1] in valid and m[pos.x1][pos.y0 - 1] in valid:  # Check left
                type10 = Pos()
                type10.x0 = pos.x0
                type10.y0 = pos.y0 - 1
                type10.x1 = pos.x1
                type10.y1 = pos.y0 - 1
                moves.append(type10)
        if pos.x0 - 1 >= 0:
            if m[pos.x0 - 1][pos.y0] in valid:  # Check up
                type11 = Pos()
                type11.x0 = pos.x0 - 1
                type11.y0 = pos.y0
                type11.x1 = None
                type11.y1 = None
                moves.append(type11)

        if len(m) - 1 >= pos.x0 + 2:
            if m[pos.x0 + 2][pos.y0] in valid:  # Check down
                type12 = Pos()
                type12.x0 = pos.x0 + 2
                type12.y0 = pos.y0
                type12.x1 = None
                type12.y1 = None
                moves.append(type12)

    return moves

# Find_pos function moves around array until it finds the current
# state of the block and then returns the coordinates.


def find_pos(m):

    pos = Pos()
    for i in range(len(m)):
        for j in range(len(m[0])):
            if m[i][j] == 'S':
                if pos.x0 is None:
                    pos.x0 = i
                    pos.y0 = j
                elif pos.x1 is None:
                    pos.x1 = i
                    pos.y1 = j
    return pos

# This just the Manhattan distance to our goal since we
# can't really draw a straight line to the goal.


def heuristic(pos, goal):
    if pos.x1 is not None and pos.y1 is not None:
        distance = (abs(goal.x0 - (pos.x0 + pos.x1)/2.0) + abs(goal.y0 - (pos.y0 + pos.y1)/2.0))*0.25
    else:
        distance = (abs(goal.x0-pos.x0)+abs(goal.y0-pos.y0))*0.25
    # if pos.x1 is not None and pos.y1 is not None:
    #     distance = min(abs(goal.x0 - pos.x0) + abs(goal.y0 - pos.y0), abs(goal.x0 - pos.x1) + abs(goal.y0 - pos.y1))
    # else:
    #     distance = abs(goal.x0 - pos.x0) + abs(goal.y0 - pos.y0)

    return distance

# In our Astar function algorithm the only diffrent thing is
# the use of a heuristic function. When we get the possible locations
# from the successor state function. We sort the successor states according to
# their Manhattan distance to the goal and continue expanding the nodes
# through that way.


def Astar(m):
    goal = goal_pos(m)
    hasSolution = False
    iterationss=0
    stepcost=0

    if goal.x0 is not None and goal.y0 is not None:
        visited = {}
        initial = find_pos(m)
        visited[initial] = 0
        path = []
        q = []
        q.append(initial)
        path.append(initial)

        while len(q) > 0:

            curr_pos = q.pop(0)
            visited[curr_pos] = 0
            path.append(curr_pos)
            curr_pos.path.append([curr_pos.x0, curr_pos.y0, curr_pos.x1, curr_pos.y1])
            if curr_pos == goal:
                hasSolution = True
                print(curr_pos.path)
                break
            positions = possible_moves(curr_pos, m)
            for pos in positions:
                pos.dist = heuristic(pos, goal)
                pos.dist +=stepcost


            #   Sort the positions according to their distances
            # positions = sorted(positions, key=lambda x: x.dist)
            valid_positions = 0
            for pos in positions:
                if pos not in visited:
                    iterationss += 1
                    pos.path.extend(curr_pos.path)
                    visited[pos] = 0
                    q.append(pos)
                    valid_positions += 1
            q = sorted(q, key=lambda x: x.dist)
            stepcost += 1
            # Remove the position if there are no ways to reach other cells
            if valid_positions == 0:
                path.pop(-1)
    print(iterationss)
    if not hasSolution:
        print("There is no solution!")

# We append the initial position to the queue. Then we pop it and store it to
# get its successor. After getting its successor we put them into the queue and
# check their successors and so on.


def BFS(m):
    goal = goal_pos(m)
    hasSolution = False
    iterations=0

    if goal.x0 is not None and goal.y0 is not None:
        visited = {}
        initial = find_pos(m)
        visited[initial] = 0
        path = []
        q = []
        q.append(initial)
        path.append(initial)

        while len(q) > 0:
            curr_pos = q.pop(0)
            visited[curr_pos] = 0
            path.append(curr_pos)
            curr_pos.path.append([curr_pos.x0, curr_pos.y0, curr_pos.x1, curr_pos.y1])
            if curr_pos == goal:
                hasSolution = True
                print(curr_pos.path)
                break
            positions = possible_moves(curr_pos, m)
            valid_positions = 0
            for pos in positions:
                # Extend the path is not visited
                if pos not in visited:
                    iterations += 1
                    pos.path.extend(curr_pos.path)
                    visited[pos] = 0
                    q.append(pos)
                    valid_positions += 1
            # remove position if there are no ways to reach other cells
            if valid_positions == 0:
                path.pop(-1)


    print(iterations)
    if not hasSolution:
        print("There is no solution!")


def main():
    # m = ([
    #     ['O', 'O', 'O', 'X', 'X', 'X', 'X', 'X', 'X', 'X'],
    #     ['O', 'O', 'O', 'O', 'O', 'O', 'X', 'X', 'X', 'X'],
    #     ['G', 'O', 'O', 'S', 'O', 'O', 'O', 'O', 'O', 'X'],
    #     ['X', 'O', 'O', 'S', 'O', 'O', 'O', 'O', 'O', 'O'],
    #     ['X', 'X', 'X', 'X', 'X', 'O', 'O', 'O', 'O', 'O'],
    #     ['X', 'X', 'X', 'X', 'X', 'X', 'O', 'O', 'O', 'X'],
    #
    # ])

    m = ([
    ['O', 'O', 'O', 'X', 'X', 'X', 'X', 'X', 'X', 'X'],
    ['O', 'O', 'O', 'O', 'O', 'O', 'X', 'X', 'X', 'X'],
    ['G', 'O', 'O', 'S', 'O', 'O', 'O', 'O', 'O', 'X'],
    ['X', 'O', 'O', 'S', 'O', 'O', 'O', 'O', 'O', 'O'],
    ['X', 'X', 'X', 'X', 'X', 'O', 'O', 'O', 'O', 'O'],
    ['X', 'X', 'X', 'X', 'X', 'X', 'O', 'O', 'O', 'X'],
])


    start = time.time()
    BFS(m)
    end = time.time() - start
    print(end*1000)
    start = time.time()
    Astar(m)
    end = time.time() - start
    print(end*1000)

if __name__ == "__main__":
    main()
