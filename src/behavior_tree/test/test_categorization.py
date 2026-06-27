from behavior_tree.PickAndPlace.categorization import classify_destination, Destination

CUTLERY = ["fork", "knife", "spoon"]
TABLEWARE = ["plate", "mug", "cup", "bowl"]
TRASH = ["paper cup"]


def _classify(label, category_map=None):
    return classify_destination(
        label, cutlery=CUTLERY, tableware=TABLEWARE, trash=TRASH,
        category_map=category_map or {},
    )


def test_cutlery_routes_to_wash_staging():
    d = _classify("fork")
    assert d.klass == "wash_staging" and d.reference_label == ""


def test_tableware_routes_to_wash_staging():
    assert _classify("plate").klass == "wash_staging"
    assert _classify("mug").klass == "wash_staging"


def test_designated_trash_routes_to_trash():
    d = _classify("paper cup")
    assert d.klass == "trash" and d.reference_label == ""


def test_unknown_routes_to_cabinet_with_label_reference():
    d = _classify("pringles")
    assert d.klass == "cabinet" and d.reference_label == "pringles"


def test_category_map_groups_label():
    d = _classify("pringles", category_map={"pringles": "snacks"})
    assert d.klass == "cabinet" and d.reference_label == "snacks"


def test_category_map_default_to_label_when_empty():
    # v1 empty map => each label its own group (+20 grouping best-effort).
    assert _classify("oats").reference_label == "oats"


def test_bowl_is_tableware_during_cleanup():
    # Bowl disambiguation: during cleanup a detected bowl is tableware; the
    # breakfast phase handles the bowl out-of-band (never via this fn).
    assert _classify("bowl").klass == "wash_staging"


def test_returns_destination_namedtuple():
    d = _classify("fork")
    assert isinstance(d, Destination)
    assert d._fields == ("klass", "reference_label")


def test_case_and_whitespace_insensitive():
    assert _classify("  Fork ").klass == "wash_staging"
