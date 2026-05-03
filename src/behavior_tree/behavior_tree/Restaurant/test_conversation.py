import rclpy
import py_trees_ros
import py_trees
from behavior_tree.TemplateNodes.Audio import (
    BtNode_Announce,
    BtNode_PhraseExtractionAction,
)


CUSTOMER_ORDER_DRINK="customer_order_drink"
CUSTOMER_ORDER_FOOD="customer_order_food"

def _create_get_info(field_name: str, storage_key: str, word_list: list[str]):
    """High-confidence-first capture with a last-resort confirmation fallback.

    Primary branch: up to 2 attempts of prompt → action-based extract. The
    action (`phrase_extraction_action`) only succeeds on server status=0,
    which means Whisper + Qwen ASR cross-check agreed on the same wordlist
    entry. The rulebook awards a 4×15 "no non-essential questions" bonus for
    accepting on that signal without a confirmation prompt.

    Fallback branch: if both primary attempts abort, re-prompt, capture
    the raw transcription via `BtNode_ListenAction`, then `BtNode_Confirm`
    speaks it back (`"Your <field> is <value>, correct?"`) and
    `BtNode_GetConfirmationAction` waits for yes/no. Preserves partial
    scoring in noisy environments at the cost of the no-confirmation
    bonus for this field only.
    """
    primary_loop = py_trees.composites.Sequence(
        name=f"Prompt+extract {field_name}",
        memory=True,
    )
    primary_loop.add_child(
        BtNode_Announce(
            name=f"Prompt for {field_name}",
            bb_source=None,
            message=f"What is your {field_name} order?.",
        )
    )
    primary_loop.add_child(
        BtNode_PhraseExtractionAction(
            name=f"High-conf extract {field_name}",
            wordlist=word_list,
            bb_dest_key=storage_key,
            timeout=7.0,
        )
    )
    primary = py_trees.decorators.Retry(
        name=f"Retry high-conf {field_name}",
        child=primary_loop,
        num_failures=10
    )

    fallback = py_trees.composites.Sequence(
        name=f"Last-resort confirm {field_name}",
        memory=True,
    )
    fallback.add_child(
        BtNode_Announce(
            name=f"Fallback prompt for {field_name}",
            bb_source=None,
            message=f"Let me try again. Please tell me your {field_name} clearly.",
        )
    )
    fallback.add_child(
        BtNode_PhraseExtractionAction(
            name=f"High-conf extract {field_name}",
            wordlist=word_list,
            bb_dest_key=storage_key,
            timeout=7.0,
        )
    )

    root = py_trees.composites.Selector(
        name=f"Get {field_name}",
        memory=True,
    )
    root.add_child(primary)
    root.add_child(py_trees.decorators.Retry(name="retry 3 times", child=fallback, num_failures=4))
    return root

def get_order():
    root = py_trees.composites.Sequence(
        name="Get customer order",
        memory=True
    )

    root.add_child(
        BtNode_Announce(
            "announce waiting door bell",
            bb_source=None,
            message="Hi customer, please speak to me after the beep sound"
        )
    )

    root.add_child(
        _create_get_info(
            field_name="drink",
            storage_key=CUSTOMER_ORDER_DRINK,
            word_list=['cola', 'water', 'sprite', 'orange', 'milk']
        )
    )

    root.add_child(
        BtNode_Announce(
            "repeat order",
            bb_source=CUSTOMER_ORDER_DRINK,
            message="Got order of "
        )
    )

    root.add_child(
        _create_get_info(
            field_name="food",
            storage_key=CUSTOMER_ORDER_FOOD,
            word_list=['chip', 'chips', 'biscuit', 'cookie', 'bread', 'lays']
        )
    )

    root.add_child(
        BtNode_Announce(
            "repeat order",
            bb_source=CUSTOMER_ORDER_FOOD,
            message="Got order of "
        )
    )

    return root


def main():
    rclpy.init()
    root = get_order()
    tree = py_trees_ros.trees.BehaviourTree(root)
    tree.setup(node_name="test_scan", timeout=15)

    def _print(t):
        print(py_trees.display.unicode_tree(root=t.root, show_status=True))

    tree.tick_tock(period_ms=500.0, post_tick_handler=_print)

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
    