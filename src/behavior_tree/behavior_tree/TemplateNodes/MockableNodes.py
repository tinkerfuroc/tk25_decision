"""
Simple node substitution for mock mode - wraps any node with WaitKeyboardPress
and handles blackboard key verification/filler creation.
"""

import py_trees as pytree
from behavior_tree.config import is_mock_mode
from behavior_tree.TemplateNodes.WaitKeyPress import BtNode_WaitKeyboardPress


def create_mockable_node(original_node, key_name="s"):
    """
    Factory function that returns a node ready for behavior tree use.
    
    If NOT in mock mode: Returns the original node unchanged.
    If in mock mode: Returns a WaitKeyboardPress node that:
        - Verifies READ blackboard keys exist
        - Creates empty filler objects for WRITE blackboard keys
    
    Args:
        original_node: The original behavior tree node instance
        key_name: Keyboard key to wait for (default: 's')
    
    Returns:
        Either the original node or a wrapped substitute node
    """
    if not is_mock_mode():
        # Not in mock mode, return original node unchanged
        return original_node
    
    # In mock mode, create a keyboard substitute with blackboard handling
    class MockSubstituteNode(BtNode_WaitKeyboardPress):
        """Extends WaitKeyboardPress to handle blackboard keys for the original node"""
        
        def __init__(self):
            super().__init__(
                name=f"🔑 Press '{key_name}' for: {original_node.name}",
                key=key_name
            )
            self.original_node = original_node
        
        def setup(self, **kwargs):
            """Setup and handle blackboard keys"""
            result = super().setup(**kwargs)
            self._handle_blackboard_keys()
            return result
        
        def _handle_blackboard_keys(self):
            """
            Inspect the original node's blackboard registrations:
            - Verify READ keys exist (warn if missing)
            - Create empty filler objects for WRITE keys
            """
            # Get blackboard clients from the original node
            if not hasattr(self.original_node, '_blackboard_clients'):
                print(f"  ℹ No blackboard clients found for {self.original_node.name}")
                return
            
            clients = self.original_node._blackboard_clients
            
            for namespace, client in clients.items():
                # Check READ keys
                if hasattr(client, '_Read') and client._Read:
                    for key in client._Read:
                        try:
                            # Try to access the key
                            _ = client.get(key)
                            print(f"  ✓ READ key verified: '{key}'")
                        except KeyError:
                            print(f"  ⚠ WARNING: READ key '{key}' missing from blackboard!")
                
                # Create fillers for WRITE keys
                if hasattr(client, '_Write') and client._Write:
                    for key in client._Write:
                        # Create simple empty object
                        class EmptyFiller:
                            """Placeholder for mock mode blackboard data"""
                            def __repr__(self):
                                return f"<MockFiller for '{key}'>"
                        
                        filler = EmptyFiller()
                        try:
                            client.set(key, filler, overwrite=True)
                            print(f"  ✓ Created filler for WRITE key: '{key}'")
                        except Exception as e:
                            print(f"  ⚠ Failed to create filler for '{key}': {e}")
    
    return MockSubstituteNode()
