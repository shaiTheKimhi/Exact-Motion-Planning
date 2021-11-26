 # Generic tree node class
class TreeNode(object):
    def __init__(self, key, val):
        self.val = val
        self.key = key 
        self.left = None
        self.right = None
        self.height = 1
    #dictionary operators for the TreeNode
    def __getitem__(self, key):
        return AVL_Tree().get(self, key)

    def __setitem__(self, key, value):
        return AVL_Tree().insert(self, key, value)


# AVL tree class which supports the
# Insert operation
class AVL_Tree(object):

    #finds value of the given key, if does not exist returns None
    def get(self, root, key):
        if not root:
            return False
        elif key == root.key:
            return root.val
        elif key < root.key:
            return self.get(root.left, key)
        else:
            return self.get(root.right, key)
 

    # Recursive function to insert key in
    # subtree rooted with node and returns
    # new root of subtree.
    def insert(self, root, key, val=0):
     
        # Perform normal BST
        if not root:
            return TreeNode(key, val)
        elif key < root.key:
            root.left = self.insert(root.left, key, val)
        else:
            root.right = self.insert(root.right, key, val)
 
        # Update the height of the
        # ancestor node
        root.height = 1 + max(self.getHeight(root.left),
                           self.getHeight(root.right))
 
        # Get the balance factor
        balance = self.getBalance(root)
 
        # If the node is unbalanced,
        # then try out the 4 cases
        # Left Left
        if balance > 1 and key < root.left.key:
            return self.rightRotate(root)
 
        # Right Right
        if balance < -1 and key > root.right.key:
            return self.leftRotate(root)
 
        # Left Right
        if balance > 1 and key > root.left.key:
            root.left = self.leftRotate(root.left)
            return self.rightRotate(root)
 
        # Right Left
        if balance < -1 and key < root.right.key:
            root.right = self.rightRotate(root.right)
            return self.leftRotate(root)

        return root
 
    def leftRotate(self, z):
 
        y = z.right
        T2 = y.left
 
        # Perform rotation
        y.left = z
        z.right = T2
 
        # Update heights
        z.height = 1 + max(self.getHeight(z.left),
                         self.getHeight(z.right))
        y.height = 1 + max(self.getHeight(y.left),
                         self.getHeight(y.right))
 
        # Return the new root
        return y
 
    def rightRotate(self, z):
 
        y = z.left
        T3 = y.right
 
        # Perform rotation
        y.right = z
        z.left = T3
 
        # Update heights
        z.height = 1 + max(self.getHeight(z.left),
                        self.getHeight(z.right))
        y.height = 1 + max(self.getHeight(y.left),
                        self.getHeight(y.right))
 
        # Return the new root
        return y
 
    def getHeight(self, root):
        if not root:
            return 0
 
        return root.height
 
    def getBalance(self, root):
        if not root:
            return 0
 
        return self.getHeight(root.left) - self.getHeight(root.right)
 
    def preOrder(self, root):
 
        if not root:
            return
 
        print("{0} ".format(root.key), end="")
        self.preOrder(root.left)
        self.preOrder(root.right)
