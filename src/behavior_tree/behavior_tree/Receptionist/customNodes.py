import py_trees
from behavior_tree.TemplateNodes.structs import Person
from behavior_tree.TemplateNodes.Audio import BtNode_Announce

from behavior_tree.messages import TextToSpeech

class BtNode_CombinePerson(py_trees.behaviour.Behaviour):
    """
    Set the specified variable on the blackboard.

    Args:
        variable_name: name of the variable to set, may be nested, e.g. battery.percentage
        variable_value: value of the variable to set
        overwrite: when False, do not set the variable if it already exists
        name: name of the behaviour
    """

    def __init__(
        self,
        name: str,
        key_dest: str,
        key_name: str,
        key_drink: str,
        key_features: str,
    ):
        super().__init__(name=name)

        self.blackboard = self.attach_blackboard_client(name=self.name)

        self.blackboard.register_key(
            key="person_name",
            access=py_trees.common.Access.READ,
            # make sure to namespace it if not already
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", key_name)
        )
        self.blackboard.register_key(
            key="drink",
            access=py_trees.common.Access.READ,
            # make sure to namespace it if not already
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", key_drink)
        )
        self.blackboard.register_key(
            key="features",
            access=py_trees.common.Access.READ,
            # make sure to namespace it if not already
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", key_features)
        )
        self.blackboard.register_key(
            key="person",
            access=py_trees.common.Access.WRITE,
            # make sure to namespace it if not already
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", key_dest)
        )
        
    def update(self):
        """
        Attempt to set the stored value in the requested blackboard variable.

        Returns:
             :data:`~py_trees.common.Status.FAILURE` if no overwrite requested
                 and the variable exists,  :data:`~py_trees.common.Status.SUCCESS` otherwise
        """
        new_person = Person()
        new_person.name = self.blackboard.person_name
        new_person.fav_drink = self.blackboard.drink
        new_person.features = self.blackboard.features
        print(new_person.name, new_person.fav_drink, new_person.features)
        self.feedback_message = f"name: {new_person.name}, drink: {new_person.fav_drink}, features: {new_person.features}"
        persons = []
        if self.blackboard.person is not None:
            persons = self.blackboard.person
        
        persons.append(new_person)
        self.blackboard.person = persons

        return py_trees.common.Status.SUCCESS
        

class BtNode_Introduce(BtNode_Announce):
    def __init__(self,
                 name: str,
                 key_person: str,
                 target_id: int,
                 introduced_id: int,
                 service_name: str = "announce",
                 describe_introduced=False
                 ):
        super(BtNode_Announce, self).__init__(name, service_name, TextToSpeech)
        self.introduced_id = introduced_id
        self.target_id = target_id
        self.describe_introduced = describe_introduced
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="persons",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", key_person)
        )
    
    def setup(self, **kwargs):
        self.announce_msg = "foo"
        return super().setup(**kwargs)
    
    def initialise(self):
        self.announce_msg = "Hello " + self.blackboard.persons[self.target_id].name + ", "
        introduced_person : Person = self.blackboard.persons[self.introduced_id]
        self.announce_msg += "Here is " + introduced_person.name + \
              " whose favorite drink is " + introduced_person.fav_drink + "."
        if self.describe_introduced:
            self.announce_msg += " " + introduced_person.features

        return super().initialise()

class BtNode_Confirm(BtNode_Announce):
    def __init__(self,
                 name: str,
                 key_confirmed: str,
                 type: str,
                 service_name: str = "announce"
                 ):
        super(BtNode_Announce, self).__init__(name=name, service_name=service_name, service_type=TextToSpeech)
        self.type = type
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="confirm_target",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", key_confirmed)
        )
    
    def setup(self, **kwargs):
        self.announce_msg = "foo"
        return super().setup(**kwargs)
    
    def initialise(self):
        # self.announce_msg = "Your " + self.type + " is " + self.blackboard.confirm_target + ". Am I correct?"
        self.announce_msg = "Your " + self.type + " is " + self.blackboard.confirm_target + ", correct?"

        return super().initialise()
