import behavior_tree.PickAndPlace.config as c


def test_action_name():
    assert c.SCAN_AND_PLACE_ACTION_NAME == "scan_and_place_action"


def test_placement_mode_constants():
    assert (c.PLACEMENT_MODE_FREE_SPACE, c.PLACEMENT_MODE_NEAR_SIMILAR,
            c.PLACEMENT_MODE_FIXED_POINT, c.PLACEMENT_MODE_NONE) == (0, 1, 2, 255)


def test_extra_surface_symbols():
    assert c.KEY_POSE_EXTRA_SURFACE == "pp_pose_extra_surface"
    assert c.KEY_POINT_EXTRA_SURFACE == "pp_point_extra_surface"
    assert c.POSE_EXTRA_SURFACE is not None
    assert c.POINT_EXTRA_SURFACE is not None


def test_category_map_is_dict():
    assert isinstance(c.CATEGORY_MAP, dict)


def test_budgets():
    assert (c.TABLE_BUDGET_SEC, c.BREAKFAST_BUDGET_SEC, c.EXTRA_BUDGET_SEC) == (200.0, 110.0, 60.0)


def test_destination_routing_shape():
    assert set(c.DESTINATION_ROUTING) == {"wash_staging", "cabinet", "trash"}
    for klass, tup in c.DESTINATION_ROUTING.items():
        assert len(tup) == 4, klass
    nav, arm, mode, point = c.DESTINATION_ROUTING["wash_staging"]
    assert (nav, arm, mode, point) == (
        c.KEY_POSE_WASH_STAGING, c.KEY_ARM_WASH,
        c.PLACEMENT_MODE_FREE_SPACE, c.KEY_POINT_WASH_STAGING)
    nav, arm, mode, point = c.DESTINATION_ROUTING["cabinet"]
    assert (nav, arm, mode, point) == (
        c.KEY_POSE_CABINET, c.KEY_ARM_CABINET,
        c.PLACEMENT_MODE_NEAR_SIMILAR, c.KEY_POINT_CABINET_DEFAULT)
    nav, arm, mode, point = c.DESTINATION_ROUTING["trash"]
    assert mode == c.PLACEMENT_MODE_NONE and point is None
