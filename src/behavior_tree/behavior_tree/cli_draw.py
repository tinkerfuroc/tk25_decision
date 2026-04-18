from behavior_tree.runtime import draw_tree


def draw_receptionist():
    from behavior_tree.Receptionist.receptionist import createReceptionist

    draw_tree(createReceptionist)


def draw_follow():
    from behavior_tree.HelpMeCarry.Track import createFollowPerson

    draw_tree(createFollowPerson)


def draw_serve_breakfast():
    from behavior_tree.ServeBreakfast.serve_breakfast import (
        createServeBreakfast,
    )

    draw_tree(createServeBreakfast)


def draw_storing_groceries():
    from behavior_tree.StoringGroceries.storing_groceries import (
        createStoreGroceries,
    )

    draw_tree(createStoreGroceries)


def draw_storing_groceries_placing_only():
    from behavior_tree.StoringGroceries.storing_groceries_place_only import (
        createStoreGroceriesPlaceOnly,
    )

    draw_tree(createStoreGroceriesPlaceOnly)


def draw_gpsr():
    from behavior_tree.GPSR.gpsr_new import createGPSR

    draw_tree(createGPSR)


def draw_yanglaozhucan():
    from behavior_tree.yanglaozhucan.yanglaozhucan import createYanglaozhucan

    draw_tree(createYanglaozhucan)


def draw_restaurant():
    from behavior_tree.Restaurant.restaurants import createRestaurantTask

    draw_tree(createRestaurantTask)


def main():
    draw_receptionist()
