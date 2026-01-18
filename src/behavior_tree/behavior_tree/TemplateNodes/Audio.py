import py_trees as pytree
from py_trees.common import Status

# from tinker_decision_msgs.srv import Announce, WaitForStart
from behavior_tree.messages import TTSCnRequest, TextToSpeech, WaitForStart, PhraseExtraction, GetConfirmation, Listen, CompareInterest, GraspRequest

from .BaseBehaviors import ServiceHandler


from typing import Optional

class BtNode_TTSCN(ServiceHandler):
    """
    Node for making a Chinese audio announcement (Text-to-Speech in Chinese).
    Returns SUCCESS once the TTS CN service finishes speaking.
    """
    def __init__(self, 
                 name: str,
                 bb_source: str,
                 service_name: str = "ttscn_service",
                 message: Optional[str] = None):
        """
        Args:
            name: Name of the node (to be displayed in the tree).
            bb_source: Blackboard key for retrieving the Chinese text announcement message.
            service_name: Name of the TTS CN service.
            message: Optional message, if given, skips reading from the blackboard.
        """
        # Call parent initializer
        super(BtNode_TTSCN, self).__init__(name, service_name, TTSCnRequest)
        
        # Store parameters
        self.bb_source = bb_source
        self.bb_read_client = None
        self.given_msg = message
        
        if self.bb_source is not None:
            self.blackboard = self.attach_blackboard_client(name=self.name)
            self.blackboard.register_key(
                key="announcement_msg",
                access=pytree.common.Access.READ,
                remap_to=pytree.blackboard.Blackboard.absolute_name("/", self.bb_source)
            )

    def setup(self, **kwargs):
        super().setup(**kwargs)
        
        # If no announcement message is given, set up a blackboard client to read from given key
        if self.bb_source is not None:
            self.logger.debug(f"Setup TTS_CN, reading from {self.bb_source}")
        else:
            self.logger.debug(f"Setup TTS_CN for message {self.given_msg}")

    def initialise(self):
        super().initialise()

        self.announce_msg = self.given_msg

        # Handle mock mode
        if self.mock_mode:
            if self.bb_source is not None:
                try:
                    read_msg = self.blackboard.announcement_msg
                    if isinstance(read_msg, str):
                        if self.given_msg is not None:
                            self.announce_msg = self.given_msg + " " + read_msg
                        else:
                            self.announce_msg = read_msg
                except:
                    pass
            self.feedback_message = f"MOCK: TTS_CN announcement '{self.announce_msg}'"
            print(f"🔊 MOCK TTS_CN: '{self.announce_msg}'")
            return

        # If no announcement message is given, read from the blackboard and verify the information
        if self.bb_source is not None:
            try:
                # Read message from the blackboard
                read_msg = self.blackboard.announcement_msg
                assert isinstance(read_msg, str)
                if self.given_msg is not None:
                    self.announce_msg = self.given_msg + " " + read_msg
                else:
                    self.announce_msg = read_msg
                
            except Exception as e:
                self.feedback_message = f"TTS_CN reading message failed"
                raise e

        # Initialize a request and set the Chinese announcement message
        request = TTSCnRequest.Request()
        request.input_text = self.announce_msg

        # Send request to TTS CN service and store the returned Future object
        self.response = self.call_service_async(request)

        # Update feedback message
        self.feedback_message = f"Initialized TTS_CN for message: {self.announce_msg}"

    def update(self) -> Status:
        # Handle mock mode
        if self.mock_mode:
            return self.wait_for_keypress_in_mock()
            
        if self.response is None:
            return pytree.common.Status.FAILURE
            
        self.logger.debug(f"Update TTS_CN: {self.announce_msg}")
        
        # If the service call is done, check its status
        if self.response.done():
            # If status is 0, all is well, return success
            if self.response.result().status == 0:
                self.feedback_message = f"Finished announcing in Chinese: {self.announce_msg}"
                return pytree.common.Status.SUCCESS
            else:
                # Update feedback message with the error and return failure
                self.feedback_message = f"TTS_CN for {self.announce_msg} failed with error code {self.response.result().status}"
                return pytree.common.Status.FAILURE
        else:
            # If service call is still in progress, return running
            self.feedback_message = "Still announcing in Chinese..."
            return pytree.common.Status.RUNNING


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
        self.given_msg = message
        # print(self.announce_msg)

        if self.bb_source is not None:
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
        if self.bb_source is not None:
            # self.bb_read_client = self.attach_blackboard_client(name="Announce Read")
            # self.bb_read_client.register_key("/" + self.bb_source, access=pytree.common.Access.READ)
            self.logger.debug(f"Setup Announce, reading from {self.bb_source}")
        else:
            self.logger.debug(f"Setup Announce for message {self.given_msg}")
    
    def initialise(self):
        super().initialise()

        self.announce_msg = self.given_msg

        # if no announcement message is given, read from the blackboard and verify the information
        if self.bb_source is not None:
            try:
                # self.announce_msg = self.bb_read_client.get(self.bb_source)
                read_msg = self.blackboard.announcement_msg
                assert isinstance(read_msg, str)
                if self.given_msg is not None:
                    self.announce_msg = self.given_msg + " " + read_msg
                else:
                    self.announce_msg = read_msg
                
            except Exception as e:
                self.feedback_message = f"Announce reading message failed"
                raise e

        # Handle mock mode
        if self.mock_mode:
            self.feedback_message = f"MOCK: Would announce '{self.announce_msg}'"
            print(f"🔊 MOCK ANNOUNCEMENT: {self.announce_msg}")
            return

        # initialize a request and set the annnouncement message
        request = TextToSpeech.Request()
        request.text = self.announce_msg

        # send request to service and store the returned Future object
        self.response = self.call_service_async(request)

        # update feedback message
        self.feedback_message = f"Initialized Announce for message {self.announce_msg}"

    def update(self) -> Status:
        # Handle mock mode
        if self.mock_mode:
            return self.wait_for_keypress_in_mock()
        
        # Check if response exists (should always exist in non-mock mode)
        if self.response is None:
            self.feedback_message = "No response object"
            return pytree.common.Status.FAILURE
            
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
        # Handle mock mode
        if self.mock_mode:
            self.feedback_message = "MOCK: Wait for start signal received"
            print(f"🎬 MOCK WAIT FOR START: Signal received")
            return
            
        request = WaitForStart.Request()
        request.timeout = 15.0
        self.response = self.call_service_async(request)
        self.feedback_message = f"Initialized wait for start"

    def update(self) -> Status:
        # Handle mock mode
        if self.mock_mode:
            return self.wait_for_keypress_in_mock()
            
        if self.response is None:
            return pytree.common.Status.FAILURE
            
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


class BtNode_GraspRequest(ServiceHandler):
    """
    Node to handle grasp request using speech input, returns success once the object is successfully grasped.
    """
    def __init__(self, 
                 name: str,
                 object_names: list,
                 object_codes: list,
                 bb_dest_key: str,
                 service_name: str = "grasp_request",  # Corrected service name
                 timeout: float = 15.0
                 ):
        """
        Args:
            name: Name of the node (to be displayed in the tree).
            object_names: List of object names that the system can grasp.
            object_codes: List of object codes corresponding to object names.
            bb_dest_key: Blackboard key for storing the matched object code.
            service_name: Name of the ROS service (default is "grasp_request").
            timeout: Timeout for the grasp request, defaults to 15 seconds.
        """
        # Call parent initializer
        super(BtNode_GraspRequest, self).__init__(name, service_name, GraspRequest)
        
        # Store parameters
        self.object_names = object_names
        self.object_codes = object_codes
        self.bb_dest_key = bb_dest_key
        
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="matched_object_code",
            access=pytree.common.Access.WRITE,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", self.bb_dest_key)
        )
        
        self.timeout = timeout
        self.response = None
    
    def initialise(self):
        """
        Initializes the grasp request and sends the request to the service.
        """
        # Handle mock mode
        if self.mock_mode:
            import random
            mock_code = random.choice(self.object_codes) if self.object_codes else "mock_object_001"
            self.blackboard.matched_object_code = mock_code
            self.feedback_message = f"MOCK: Grasped object with code {mock_code}"
            print(f"🎯 MOCK GRASP REQUEST: Selected '{mock_code}' from {self.object_names}")
            return
            
        # Create request and populate object names and codes
        request = GraspRequest.Request()
        request.object_names = self.object_names
        request.object_codes = self.object_codes
        
        # Send the request to the service
        self.response = self.call_service_async(request)
        
        self.feedback_message = f"Initialized grasp request for objects: {', '.join(self.object_names)}"
    
    def update(self) -> Status:
        """
        Updates the status of the grasp request.
        Returns SUCCESS once the object is successfully grasped, FAILURE if there is an error.
        """
        # Handle mock mode
        if self.mock_mode:
            return self.wait_for_keypress_in_mock()
            
        if self.response is None:
            return pytree.common.Status.FAILURE
            
        if self.response.done():
            if self.response.result().status == 0:
                # Successful grasp
                matched_object_code = self.response.result().matched_object_code
                self.blackboard.matched_object_code = matched_object_code
                self.feedback_message = f"Successfully grasped object with code: {matched_object_code}"
                return pytree.common.Status.SUCCESS
            else:
                # Failure in grasp
                self.feedback_message = f"Grasp request failed with error: {self.response.result().error_message}"
                return pytree.common.Status.FAILURE
        else:
            # Still waiting for the response
            self.feedback_message = "Still processing grasp request..."
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
        # Handle mock mode
        if self.mock_mode:
            # In mock mode, randomly select a word from the wordlist
            import random
            mock_result = random.choice(self.wordlist) if self.wordlist else "mock_phrase"
            self.blackboard.phrase = mock_result
            self.feedback_message = f"MOCK: Extracted phrase '{mock_result}'"
            print(f"🎤 MOCK PHRASE EXTRACTION: '{mock_result}' (from wordlist: {self.wordlist[:3]}...)")
            return
        
        request = PhraseExtraction.Request()
        request.timeout = self.timeout
        request.wordlist = self.wordlist
        self.response = self.call_service_async(request)
        self.feedback_message = f"Initialized phrase extraction"

    def update(self) -> Status:
        # Handle mock mode
        if self.mock_mode:
            return self.wait_for_keypress_in_mock()
        
        # Check if response exists
        if self.response is None:
            self.feedback_message = "No response object"
            return pytree.common.Status.FAILURE
            
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
        # Handle mock mode
        if self.mock_mode:
            mock_target = "mock_target_phrase"
            self.blackboard.target = mock_target
            self.feedback_message = f"MOCK: Extracted target '{mock_target}'"
            print(f"🎯 MOCK TARGET EXTRACTION: '{mock_target}'")
            return
            
        request = PhraseExtraction.Request()
        request.timeout = self.timeout
        request.wordlist = []
        self.response = self.call_service_async(request)
        self.feedback_message = f"Initialized target extraction"

    def update(self) -> Status:
        # Handle mock mode
        if self.mock_mode:
            return self.wait_for_keypress_in_mock()
            
        if self.response is None:
            return pytree.common.Status.FAILURE
            
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
        self._mock_pressed = False
        self._mock_input_thread = None
        self._mock_stop_thread = False
        self._mock_old_settings = None
    
    def _read_key_thread(self):
        """Background thread to read keyboard input in mock mode."""
        import sys
        while not self._mock_stop_thread and not self._mock_pressed:
            try:
                ch = sys.stdin.read(1)
                if ch:
                    self._mock_pressed = True
            except Exception:
                break
    
    def setup(self, **kwargs):
        super().setup(**kwargs)

        self.logger.debug(f"Setup getting confirmation")
    
    def initialise(self):
        # Handle mock mode - setup keyboard listener
        if self.mock_mode:
            import sys
            import termios
            import tty
            import threading
            
            self._mock_pressed = False
            self._mock_stop_thread = False
            
            # Save old terminal settings and set to cbreak mode
            self._mock_old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
            
            self.feedback_message = f"MOCK: Press any key to confirm"
            print(f"✅ MOCK GET CONFIRMATION: Press any key to confirm (will return SUCCESS)")
            
            # Start background thread to read keyboard input
            self._mock_input_thread = threading.Thread(target=self._read_key_thread, daemon=True)
            self._mock_input_thread.start()
            return
            
        request = GetConfirmation.Request()
        request.timeout = self.timeout
        self.response = self.call_service_async(request)
        self.feedback_message = f"Initialized get confirmation"

    def update(self) -> Status:
        # Handle mock mode - wait for keyboard press
        if self.mock_mode:
            if self._mock_pressed:
                return pytree.common.Status.SUCCESS
            else:
                return pytree.common.Status.RUNNING
            
        if self.response is None:
            return pytree.common.Status.FAILURE
            
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
    
    def terminate(self, new_status: Status) -> None:
        """Restore terminal settings when behavior terminates in mock mode."""
        if self.mock_mode:
            import sys
            import termios
            
            self._mock_stop_thread = True
            if self._mock_old_settings is not None:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._mock_old_settings)
                self._mock_old_settings = None
            if self._mock_input_thread is not None:
                self._mock_input_thread.join(timeout=0.1)
                self._mock_input_thread = None
        

class BtNode_Listen(ServiceHandler):
    def __init__(self,
                 name: str,
                 bb_dest_key: str,
                 service_name = "listen_service",
                 timeout : float = 5.0
                 ):
        super().__init__(name, service_name, Listen)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="message",
            access=pytree.common.Access.WRITE,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_dest_key)
        )
        self.timeout = timeout
    
    def initialise(self):
        # Handle mock mode
        if self.mock_mode:
            mock_message = "This is a mock speech input message"
            self.blackboard.message = mock_message
            self.feedback_message = f"MOCK: Listened and got '{mock_message}'"
            print(f"👂 MOCK LISTEN: '{mock_message}'")
            return
            
        request = Listen.Request()
        request.timeout = self.timeout
        self.response = self.call_service_async(request)
        self.feedback_message = f"Initialized listen"
    
    def update(self):
        # Handle mock mode
        if self.mock_mode:
            return self.wait_for_keypress_in_mock()
            
        if self.response is None:
            return Status.FAILURE
            
        self.logger.debug(f"Update listen")
        if self.response.done():
            if self.response.result().status == 0:
                self.feedback_message = "got command"
                self.blackboard.message = self.response.result().message
                return Status.SUCCESS
            else:
                self.feedback_message = f"Listen failed with error code {self.response.result().status}: {self.response.result().error_message}"
                return Status.FAILURE
        else:
            self.feedback_message = "Still listening..."
            return Status.RUNNING

class BtNode_CompareInterest(ServiceHandler):
    def __init__(self,
                 name: str,
                 bb_source_key1: str,
                 bb_source_key2: str,
                 bb_dest_key: str,
                 service_name = "compare_interest_service",
                 timeout : float = 5.0
                 ):
        super().__init__(name, service_name, CompareInterest)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="first_statement",
            access=pytree.common.Access.READ,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_source_key1)
        )
        self.blackboard.register_key(
            key="second_statement",
            access=pytree.common.Access.READ,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_source_key2)
        )
        self.blackboard.register_key(
            key="result",
            access=pytree.common.Access.WRITE,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_dest_key)
        )
        self.timeout = timeout

    def initialise(self):
        # Handle mock mode
        if self.mock_mode:
            mock_interest = "robotics and artificial intelligence"
            self.blackboard.result = mock_interest
            self.feedback_message = f"MOCK: Common interest is '{mock_interest}'"
            print(f"🤝 MOCK COMPARE INTEREST: Found '{mock_interest}'")
            return
            
        request = CompareInterest.Request()
        request.first_statement = self.blackboard.first_statement
        request.sec_statement = self.blackboard.second_statement
        self.response = self.call_service_async(request)
        self.feedback_message = f"Initialized Compare Interest"
    
    def update(self):
        # Handle mock mode
        if self.mock_mode:
            return self.wait_for_keypress_in_mock()
            
        if self.response is None:
            return Status.FAILURE
            
        self.logger.debug(f"Update compare interest")
        if self.response.done():
            if self.response.result().status == 0:
                self.blackboard.result = self.response.result().common_interest
                self.logger.debug(f"Compare interest result: {self.blackboard.result}")
                self.feedback_message = f"Compare interest result: {self.response.result().common_interest}"
                return Status.SUCCESS
            else:
                self.feedback_message = f"Compare interest failed with error code {self.response.result().status}: {self.response.result().error_message}"
                return Status.FAILURE
        else:
            self.feedback_message = "Still comparing interest..."
            return Status.RUNNING

