"""Python port of TreeStruct.cs.

This module keeps the original C# naming style (`Tree`, `Node`, `addNode`,
`getParent`, etc.) so existing code translated from DMArmDLL can call it with
minimal changes.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import List

import numpy as np


@dataclass
class Node:
    leftChild: int = -1
    rightChild: int = -1
    parent: int = -1
    isEmpty: bool = True
    q: float = 0.0
    sin: float = 0.0
    cos: float = 0.0


class Tree:
    """Fixed-capacity binary tree used by Robot.ikine8()."""

    def __init__(self, rootNode: float, capacity: int = 256) -> None:
        self.node: List[Node] = [Node() for _ in range(capacity)]
        self.node[0].q = float(rootNode)
        self.node[0].sin = math.sin(rootNode)
        self.node[0].cos = math.cos(rootNode)
        self.node[0].isEmpty = False
        self.nodeNum = 1

    def arrayOfTree(self) -> np.ndarray:
        treeArray = np.zeros((self.nodeNum, 6), dtype=float)
        for i in range(self.nodeNum):
            treeArray[i, 0] = self.node[i].leftChild
            treeArray[i, 1] = self.node[i].rightChild
            treeArray[i, 2] = self.node[i].parent
            treeArray[i, 3] = self.node[i].q
            treeArray[i, 4] = self.node[i].sin
            treeArray[i, 5] = self.node[i].cos
        return treeArray

    def addNode(self, fatherNode: int, nodeValue: float) -> int:
        """Add a node as the first available left/right child of fatherNode."""
        newNode = -1
        for i, node in enumerate(self.node):
            if node.isEmpty:
                node.q = float(nodeValue)
                node.sin = math.sin(nodeValue)
                node.cos = math.cos(nodeValue)
                node.isEmpty = False
                newNode = i
                break

        if newNode < 0:
            return -1

        father = self.node[fatherNode]
        if father.leftChild == -1:
            father.leftChild = newNode
            self.node[newNode].parent = fatherNode
            self.nodeNum += 1
        elif father.rightChild == -1:
            father.rightChild = newNode
            self.node[newNode].parent = fatherNode
            self.nodeNum += 1
        else:
            return -1
        return newNode

    def getParent(self, nodeIndex: int, generation: int = 1) -> int:
        """Return the parent node index; generation=0 returns nodeIndex itself."""
        parentIndex = nodeIndex
        for _ in range(generation):
            parentIndex = self.node[parentIndex].parent
        return parentIndex

    def getLeftChild(self, nodeIndex: int) -> int:
        return self.node[nodeIndex].leftChild

    def getRightChild(self, nodeIndex: int) -> int:
        return self.node[nodeIndex].rightChild
