import py_trees as pytree
from py_trees.common import Status

# from tinker_decision_msgs.srv import Announce, WaitForStart
from behavior_tree.messages import TextToSpeech, WaitForStart, PhraseExtraction, GetConfirmation

from .BaseBehaviors import ServiceHandler

class BtNode_Announce(ServiceHandler):
    """
    Node for making an audio announcement, returns SUCCESS once announcement finished
    """
    def __init__(self, 
                 name : str,
                 bb_source : str,
                 service_name : str = "announce",
                 message : str = None
                 ):
        """
        Args:
            name: name of the node (to be displayed in the tree)
            bb_source: blackboard key for retrieving a str announcement message
            service_name: name of the service for Announce
            message: optional message, if given, skips reading from blackboard
        """

        # call parent initializer
        super(BtNode_Announce, self).__init__(name, service_name, TextToSpeech)
        
        # store parameters
        self.bb_source = bb_source
        self.bb_read_client = None
        self.announce_msg = message
        # print(self.announce_msg)

        if self.announce_msg is None:
            self.blackboard = self.attach_blackboard_client(name=self.name)
            self.blackboard.register_key(
                key = "announcement_msg",
                access=pytree.common.Access.READ,
                remap_to=pytree.blackboard.Blackboard.absolute_name("/", self.bb_source)
            )

    
    def setup(self, **kwargs):
        print(self.service_type)
        super().setup(**kwargs)

        # print(self.announce_msg, self.name)
        # if no announcement message is given, set up a blackboard client to read from given key
        if self.announce_msg is None:
            # self.bb_read_client = self.attach_blackboard_client(name="Announce Read")
            # self.bb_read_client.register_key("/" + self.bb_source, access=pytree.common.Access.READ)
            self.logger.debug(f"Setup Announce, reading from {self.bb_source}")
        else:
            self.logger.debug(f"Setup Announce for message {self.announce_msg}")
    
    def initialise(self):
        super().initialise()

        # if no announcement message is given, read from the blackboard and verify the information
        if not self.announce_msg:
            try:
                # self.announce_msg = self.bb_read_client.get(self.bb_source)
                self.announce_msg = self.blackboard.announcement_msg
                assert isinstance(self.announce_msg, str)
            except Exception as e:
                self.feedback_message = f"Announce reading message failed"
                raise e

        # initialize a request and set the annnouncement message
        request = TextToSpeech.Request()
        request.text = self.announce_msg

        # send request to service and store the returned Future object
        self.response = self.client.call_async(request)

        # update feedback message
        self.feedback_message = f"Initialized Announce for message {self.announce_msg}"

    def update(self) -> Status:
        self.logger.debug(f"Update Announce {self.announce_msg}")
        # if the service is done, check its status
        if self.response.done():
            # if status is 0, all is well, return success
            if self.response.result().status == 0:
                self.feedback_message = f"Finished announcing {self.announce_msg}"
                return pytree.common.Status.SUCCESS
            # else, update feedback message to reflect the error message and return failure
            else:
                self.feedback_message = f"Announce for {self.announce_msg} failed with error code {self.response.result().status}"
                return pytree.common.Status.FAILURE
        # if service is not done, simply return running
        else:
            self.feedback_message = "Still announcing..."
            return pytree.common.Status.RUNNING


class BtNode_WaitForStart(ServiceHandler):
    """
    Node to wait for an audio signal to start task, returns success once signal is received
    """
    def __init__(self, 
                 name : str,
                 service_name : str = "wait_for_start"
                 ):
        super(BtNode_WaitForStart, self).__init__(name, service_name, WaitForStart)
    
    def setup(self, **kwargs):
        super().setup(**kwargs)

        self.logger.debug(f"Setup waiting for start")
    
    def initialise(self):
        request = WaitForStart.Request()
        request.timeout = 15.0
        self.response = self.client.call_async(request)
        self.feedback_message = f"Initialized wait for start"

    def update(self) -> Status:
        self.logger.debug(f"Update wait for start")
        if self.response.done():
            if self.response.result().status == 0:
                self.feedback_message = "Started"
                return pytree.common.Status.SUCCESS
            else:
                self.feedback_message = f"Wait for start failed with error code {self.response.result().status}: {self.response.result().error_message}"
                return pytree.common.Status.FAILURE
        else:
            self.feedback_message = "Still waiting for start..."
            return pytree.common.Status.RUNNING


class BtNode_PhraseExtraction(ServiceHandler):
    """
    Node to extract a phrase from a given speech, returns success once phrase is extracted
    """
    def __init__(self, 
                 name : str,
                 wordlist : list,
                 bb_dest_key : str,
                 service_name : str = "phrase_extraction_service",
                 timeout : float = 15.0
                 ):
        super(BtNode_PhraseExtraction, self).__init__(name, service_name, PhraseExtraction)

        self.wordlist = wordlist
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="phrase",
            access=pytree.common.Access.WRITE,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_dest_key)
        )
        self.timeout = timeout
        self.node = None
    
    def initialise(self):
        request = PhraseExtraction.Request()
        request.timeout = self.timeout
        request.wordlist = self.wordlist
        self.response = self.client.call_async(request)
        self.feedback_message = f"Initialized phrase extraction"

    def update(self) -> Status:
        self.logger.debug(f"Update phrase extraction")
        if self.response.done():
            if self.response.result().status == 0:
                self.feedback_message = f"Extracted phrase: {self.response.result().phrase}"
                self.blackboard.phrase = self.response.result().phrase
                return pytree.common.Status.SUCCESS
            else:
                self.feedback_message = f"Phrase extraction failed with error code {self.response.result().status}: {self.response.result().error_message}"
                return pytree.common.Status.FAILURE
        else:
            self.feedback_message = "Still extracting phrase..."
            return pytree.common.Status.RUNNING

class BtNode_TargetExtraction(ServiceHandler):
    """
    Node to extract a grasp target from a given speech, returns success once phrase is extracted
    """
    def __init__(self, 
                 name : str,
                 bb_dest_key : str,
                 service_name : str = "target_extraction_service",
                 timeout : float = 15.0
                 ):
        super(BtNode_TargetExtraction, self).__init__(name, service_name, PhraseExtraction)

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="target",
            access=pytree.common.Access.WRITE,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_dest_key)
        )
        self.timeout = timeout
        self.node = None
    
    def initialise(self):
        request = PhraseExtraction.Request()
        request.timeout = self.timeout
        request.wordlist = []
        self.response = self.client.call_async(request)
        self.feedback_message = f"Initialized target extraction"

    def update(self) -> Status:
        self.logger.debug(f"Update target extraction")
        if self.response.done():
            if self.response.result().status == 0:
                self.feedback_message = f"Extracted target: {self.response.result().phrase}"
                self.blackboard.target = self.response.result().phrase
                return pytree.common.Status.SUCCESS
            else:
                self.feedback_message = f"Target extraction failed with error code {self.response.result().status}: {self.response.result().error_message}"
                return pytree.common.Status.FAILURE
        else:
            self.feedback_message = "Still extracting target..."
            return pytree.common.Status.RUNNING

class BtNode_GetConfirmation(ServiceHandler):
    """
    Node to get confirmation from a given speech, returns success once confirmation is received
    """
    def __init__(self, 
                 name : str,
                 service_name : str = "get_confirmation_service",
                 timeout : float = 15.0
                 ):
        super(BtNode_GetConfirmation, self).__init__(name, service_name, GetConfirmation)
        self.timeout = timeout
    
    def setup(self, **kwargs):
        super().setup(**kwargs)

        self.logger.debug(f"Setup getting confirmation")
    
    def initialise(self):
        request = GetConfirmation.Request()
        request.timeout = self.timeout
        self.response = self.client.call_async(request)
        self.feedback_message = f"Initialized get confirmation"

    def update(self) -> Status:
        self.logger.debug(f"Update get confirmation")
        if self.response.done():
            if self.response.result().status == 0:
                self.feedback_message = f"Got confirmation: {self.response.result().confirmed}"
                if self.response.result().confirmed:
                    return pytree.common.Status.SUCCESS
                else:
                    return pytree.common.Status.FAILURE
            else:
                self.feedback_message = f"Get confirmation failed with error code {self.response.result().status}: {self.response.result().error_message}"
                return pytree.common.Status.FAILURE
        else:
            self.feedback_message = "Still getting confirmation..."
            return pytree.common.Status.RUNNING


