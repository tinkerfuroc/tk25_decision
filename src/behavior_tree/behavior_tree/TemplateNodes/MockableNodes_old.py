"""
Mock-aware behavior tree nodes that can substitute real actions with keyboard press.
"""
import py_trees
from behavior_tree.config import is_mock_mode
from behavior_tree.TemplateNodes.WaitKeyPress import BtNode_WaitKeyboardPress


class MockWriteBlackboard(py_trees.behaviour.Behaviour):
    """
    A node that writes a mock object to the blackboard.
    Used to fulfill blackboard write requirements in mock mode.
    """
    def __init__(self, name: str, bb_key: str, mock_value=None):
        super().__init__(name)
        self.bb_key = bb_key
        self.mock_value = mock_value if mock_value is not None else {}
        
    def setup(self, **kwargs):
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(self.bb_key, access=py_trees.common.Access.WRITE)
        
    def update(self):
        # Write mock value to blackboard
        self.blackboard.set(self.bb_key, self.mock_value, overwrite=True)
        return py_trees.common.Status.SUCCESS


def create_mock_node_with_blackboard(real_node, mock_message: str, mock_key: str = 's', bb_write_keys=None):
    """
    Create a sequence that:
    1. Waits for keyboard press
    2. Writes mock values to blackboard for any write keys
    
    Args:
        real_node: The real node to use when not in mock mode
        mock_message: Description of what the mock represents
        mock_key: Key to press for mock mode (default 's')
        bb_write_keys: Dict of {blackboard_key: mock_value} to write after keyboard press
    
    Returns:
        Either the real node or a sequence with keyboard press + blackboard writes
    """
    if not is_mock_mode():
        return real_node
    
    # Create a sequence for mock mode
    mock_sequence = py_trees.composites.Sequence(f"MOCK: {real_node.name}", memory=False)
    
    # Add keyboard press
    mock_sequence.add_child(BtNode_WaitKeyboardPress(f"MOCK: {mock_message}", key=mock_key))
    
    # Add blackboard writes for any write keys
    if bb_write_keys:
        for bb_key, mock_value in bb_write_keys.items():
            mock_sequence.add_child(MockWriteBlackboard(
                name=f"Write mock {bb_key}",
                bb_key=bb_key,
                mock_value=mock_value
            ))
    
    return mock_sequence


def create_mockable_node(real_node, mock_message: str, mock_key: str = 's'):
    """
    Create a node that can be substituted with keyboard press in mock mode.
    
    Args:
        real_node: The real node to use when not in mock mode
        mock_message: Description of what the mock represents
        mock_key: Key to press for mock mode (default 's')
    
    Returns:
        Either the real node or a keyboard press node depending on mock mode
    """
    if is_mock_mode():
        return BtNode_WaitKeyboardPress(f"MOCK: {mock_message}", key=mock_key)
    else:
        return real_node


class MockableSelector(py_trees.composites.Selector):
    """
    A selector that tries the real node first, but falls back to keyboard press
    if the node fails due to missing dependencies.
    """
    
    def __init__(self, name: str, real_node, mock_message: str, mock_key: str = 's', **kwargs):
        super().__init__(name=name, **kwargs)
        self.add_child(real_node)
        self.add_child(BtNode_WaitKeyboardPress(f"FALLBACK: {mock_message}", key=mock_key))


class MockableSequence(py_trees.composites.Sequence):
    """
    A sequence that can automatically skip or substitute nodes when in mock mode.
    """
    
    def __init__(self, name: str, memory: bool = True, **kwargs):
        super().__init__(name=name, memory=memory, **kwargs)
        self._mock_mode = is_mock_mode()
    
    def add_child(self, child, mock_message: str = None, mock_key: str = 's'):
        """
        Add a child that can be mocked.
        
        Args:
            child: The child node to add
            mock_message: If provided, wrap the child in a mockable wrapper
            mock_key: Key to use for mock mode
        """
        if mock_message and self._mock_mode:
            super().add_child(BtNode_WaitKeyboardPress(f"MOCK: {mock_message}", key=mock_key))
        else:
            super().add_child(child)


def conditional_add(parent, child, condition: bool, mock_substitute=None):
    """
    Conditionally add a child to a composite node.
    
    Args:
        parent: Parent composite node
        child: Child node to add
        condition: If True, add the child; if False, add mock_substitute
        mock_substitute: Node to add if condition is False (default: None means skip)
    """
    if condition:
        parent.add_child(child)
    elif mock_substitute is not None:
        parent.add_child(mock_substitute)
