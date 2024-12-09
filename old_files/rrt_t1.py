import numpy as np
import random
from copy import deepcopy
from lib.calculateFKJac import FK_Jac

lowerLim = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
upperLim = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])

class Tree:
    class Node:
        def __init__(self, state, parent):
            self.state = state
            self.parent = parent

    def __init__(self):
        self.nodes = []

    def __len__(self):
        return len(self.nodes)

    def add_node(self, state, parent):
        node = Tree.Node(state, parent)
        self.nodes.append(node)
        return node

    def nearest_neighbor(self, state, comparator):
        return min(self.nodes, key=lambda node: comparator(node.state, state))

    def extract_path_to_root(self, node):
        path = []
        while node.parent is not None:
            path.append(node.state)
            node = node.parent
        path.append(node.state)
        path.reverse()
        return path


def is_self_collision(q, fk_instance):
    for i in range(7):
        if q[i] < lowerLim[i] or q[i] > upperLim[i]:
            return True

    joint_pos, _ = fk_instance.forward_expanded(q)
    tolerance = 0.01
    for i in range(7):
        for j in range(i + 1, 7):
            if np.linalg.norm(joint_pos[i] - joint_pos[j]) < tolerance:
                return True

    return False


def check_if_feasible(current, new, fk_instance):
    if is_self_collision(new, fk_instance):
        return False

    num_divisions = 10
    for i in range(num_divisions):
        q = current + (new - current) * i / num_divisions
        if is_self_collision(q, fk_instance):
            return False

    return True


TRAPPED = 0
ADVANCED = 1
REACHED = 2


def steer(from_config, to_config, step_size):
    diff = to_config - from_config
    distance = np.linalg.norm(diff)

    if distance < step_size:
        return to_config, REACHED
    else:
        return from_config + (diff / distance) * step_size, ADVANCED


def extend(tree, rand_config, step_size, fk_instance):
    nearest_to_rand = tree.nearest_neighbor(rand_config, lambda x, y: np.linalg.norm(x - y))
    q_new, status = steer(nearest_to_rand.state, rand_config, step_size)
    if check_if_feasible(nearest_to_rand.state, q_new, fk_instance):
        q_new_node = tree.add_node(q_new, nearest_to_rand)
        return q_new_node, status

    return None, TRAPPED


def connect(tree, new_config, step_size, fk_instance):
    q_new, status = extend(tree, new_config, step_size, fk_instance)
    while status == ADVANCED:
        q_new, status = extend(tree, new_config, step_size, fk_instance)

    return q_new, status


def rrt(start, goal):
    path = []
    fk = FK_Jac()

    if is_self_collision(start, fk) or is_self_collision(goal, fk):
        return path

    tree_start = Tree()
    tree_start.add_node(start, None)

    goal_tree = Tree()
    goal_tree.add_node(goal, None)

    max_iterations = 500
    step_size = 0.4
    goal_bias = 0.1

    num_steps = 0
    reached = False

    while num_steps < max_iterations:
        sample = np.random.uniform(low=lowerLim, high=upperLim)

        if random.random() < 0.5:
            current_tree = tree_start
            target_tree = goal_tree
        else:
            current_tree = goal_tree
            target_tree = tree_start

        if current_tree is tree_start and random.random() < goal_bias:
            sample = goal
        elif current_tree is goal_tree and random.random() < goal_bias:
            sample = start

        q_new_node, status = extend(current_tree, sample, step_size, fk)
        if status != TRAPPED:
            connection_node, status = connect(target_tree, q_new_node.state, step_size, fk)

            if status == REACHED:
                reached = True
                path = current_tree.extract_path_to_root(q_new_node)
                path = path + list(reversed(target_tree.extract_path_to_root(connection_node)))
                if current_tree is goal_tree:
                    path = list(reversed(path))
                break

        num_steps += 1

    if not reached:
        print("No path found")
        nearest_to_goal = tree_start.nearest_neighbor(goal, lambda x, y: np.linalg.norm(x - y))
        path = tree_start.extract_path_to_root(nearest_to_goal)
    return np.array(path)

