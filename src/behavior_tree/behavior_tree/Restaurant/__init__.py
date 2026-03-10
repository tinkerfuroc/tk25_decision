def createRestaurantTask():
    from .restaurants import createRestaurantTask as _create_restaurant_task

    return _create_restaurant_task()


def restaurant():
    from .restaurants import restaurant as _restaurant

    return _restaurant()


try:
    from .custumNodes import (
        BtNode_CommunicateWithBarman,
        BtNode_ConfirmOrder,
        BtNode_DetectCallingCustomer,
        BtNode_DetectTray,
        BtNode_ServeOrder,
        BtNode_TakeOrder,
    )
except Exception:  # pragma: no cover - available in ROS runtime
    BtNode_DetectCallingCustomer = None
    BtNode_TakeOrder = None
    BtNode_ConfirmOrder = None
    BtNode_CommunicateWithBarman = None
    BtNode_DetectTray = None
    BtNode_ServeOrder = None

__all__ = [
    'createRestaurantTask',
    'restaurant',
    'BtNode_DetectCallingCustomer',
    'BtNode_TakeOrder',
    'BtNode_ConfirmOrder',
    'BtNode_CommunicateWithBarman',
    'BtNode_DetectTray',
    'BtNode_ServeOrder'
]
