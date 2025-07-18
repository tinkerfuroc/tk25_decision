import py_trees
from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WaitTicks
from behavior_tree.TemplateNodes.Audio import BtNode_Announce, BtNode_GetConfirmation

def createPromptReached():
    """
    Repeatedly asks if person has reached their destination. RUNNING if not, SUCCESS if yes.
    """
    prompt_seq = py_trees.composites.Sequence(name="check if reached", memory=True)
    root = py_trees.decorators.Retry(name="retry", child=prompt_seq, num_failures=-1)

    prompt_seq.add_child(BtNode_Announce(name="Ask if reached", bb_source=None, message="Have you reached your destination?"))
    prompt_seq.add_child(BtNode_GetConfirmation(name="get confirmation"))
    prompt_seq.add_child(BtNode_WaitTicks(name="Wait 10 ticks", ticks=10))

    return root

def testPromptReached():
    root = py_trees.composites.Sequence(name="sequence", memory=True)
    root.add_child(createPromptReached())
    root.add_child(BtNode_Announce(name="announced reached", bb_source=None, message="Confirmed reached destination."))
    root.add_child(py_trees.behaviours.Running(name="end"))
    return root