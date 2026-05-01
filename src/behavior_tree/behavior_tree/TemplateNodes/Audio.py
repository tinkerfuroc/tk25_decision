# Copyright 2025 Tinker Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#
# Audio Nodes Module
# ==================
#
# This module provides behavior tree nodes for robot audio operations.
# All nodes inherit from ServiceHandler and include built-in mock mode support.
#
# Classes
# -------
# BtNode_TTSCN
#     Text-to-speech for Chinese announcements.
# BtNode_Announce
#     Text-to-speech for English announcements.
# BtNode_WaitForStart
#     Waits for an audio start signal.
# BtNode_GraspRequest
#     Handles speech-based grasp object selection.
# BtNode_PhraseExtraction
#     Extracts phrases from speech using a wordlist.
# BtNode_TargetExtraction
#     Extracts grasp target from speech.
# BtNode_GetConfirmation
#     Gets confirmation from speech (deprecated; use BtNode_GetConfirmationAction).
# BtNode_GetConfirmationAction
#     Action-based confirmation wrapping tk_24_audio's `get_confirmation_action`.
# BtNode_Listen
#     Listens for speech input (deprecated; use BtNode_ListenAction).
# BtNode_ListenAction
#     Action-based listen wrapping tk_24_audio's `listen_action`.
# BtNode_CompareInterest
#     Compares interests between two statements for conversation matching.
#
# Mock Mode
# ---------
# All audio nodes support mock mode via mock_config.json settings.
# In mock mode, speech recognition returns simulated or random results.
#

import time
import warnings

import py_trees as pytree
from py_trees.common import Status

# from tinker_decision_msgs.srv import Announce, WaitForStart
from behavior_tree.messages import (
    TTSCnRequest,
    TextToSpeech,
    WaitForStart,
    PhraseExtraction,
    GetConfirmation,
    Listen,
    CompareInterest,
    GraspRequest,
    GetConfirmationAction,
    ListenAction,
    PhraseExtractionAction,
)

from .BaseBehaviors import ServiceHandler
from .ActionBase import ActionHandler
from behavior_tree.messages import action_msgs


from typing import Optional


class BtNode_TTSCN(ServiceHandler):
    """
    Node for making a Chinese audio announcement (Text-to-Speech in Chinese).
    Returns SUCCESS once the TTS CN service finishes speaking.
    """

    def __init__(
        self,
        name: str,
        bb_source: str,
        service_name: str = "ttscn_service",
        message: Optional[str] = None,
    ):
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
                remap_to=pytree.blackboard.Blackboard.absolute_name(
                    "/", self.bb_source
                ),
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
                self.feedback_message = (
                    f"Finished announcing in Chinese: {self.announce_msg}"
                )
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

    def __init__(
        self,
        name: str,
        bb_source: Optional[str],
        service_name: str = "announce",
        message: Optional[str] = None,
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
                key="announcement_msg",
                access=pytree.common.Access.READ,
                remap_to=pytree.blackboard.Blackboard.absolute_name(
                    "/", self.bb_source
                ),
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

    def __init__(self, name: str, service_name: str = "wait_for_start"):
        super(BtNode_WaitForStart, self).__init__(name, service_name, WaitForStart)

    def setup(self, **kwargs):
        super().setup(**kwargs)

        self.logger.debug(f"Setup waiting for start")

    def initialise(self):
        super().initialise()

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

    def __init__(
        self,
        name: str,
        object_names: list,
        object_codes: list,
        bb_dest_key: str,
        service_name: str = "grasp_request",  # Corrected service name
        timeout: float = 15.0,
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
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", self.bb_dest_key),
        )

        self.timeout = timeout
        self.response = None

    def initialise(self):
        """
        Initializes the grasp request and sends the request to the service.
        """
        super().initialise()

        # Handle mock mode
        if self.mock_mode:
            import random

            mock_code = (
                random.choice(self.object_codes)
                if self.object_codes
                else "mock_object_001"
            )
            self.blackboard.matched_object_code = mock_code
            self.feedback_message = f"MOCK: Grasped object with code {mock_code}"
            print(
                f"🎯 MOCK GRASP REQUEST: Selected '{mock_code}' from {self.object_names}"
            )
            return

        # Create request and populate object names and codes
        request = GraspRequest.Request()
        request.object_names = self.object_names
        request.object_codes = self.object_codes

        # Send the request to the service
        self.response = self.call_service_async(request)

        self.feedback_message = (
            f"Initialized grasp request for objects: {', '.join(self.object_names)}"
        )

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
                self.feedback_message = (
                    f"Successfully grasped object with code: {matched_object_code}"
                )
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
    Node to extract a phrase from a given speech, returns success once phrase is extracted.

    DEPRECATED: use BtNode_PhraseExtractionAction. The tk_24_audio package has
    migrated phrase extraction to a ROS 2 action (`phrase_extraction_action`),
    which runs Whisper + Qwen ASR cross-check and exposes the high-confidence
    path via terminal `STATUS_SUCCEEDED`. The service-based
    `phrase_extraction_service` will be retired once all task trees (GPSR,
    Receptionist, Restaurant demo, grasp-intel) have been migrated.
    """

    def __init__(
        self,
        name: str,
        wordlist: list,
        bb_dest_key: str,
        service_name: str = "phrase_extraction_service",
        timeout: float = 15.0,
    ):
        warnings.warn(
            "BtNode_PhraseExtraction is deprecated; use BtNode_PhraseExtractionAction "
            "(tk_24_audio added action-based `phrase_extraction_action`).",
            DeprecationWarning,
            stacklevel=2,
        )
        super(BtNode_PhraseExtraction, self).__init__(
            name, service_name, PhraseExtraction
        )

        self.wordlist = wordlist
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="phrase",
            access=pytree.common.Access.WRITE,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_dest_key),
        )
        self.timeout = timeout
        self.node = None

    def initialise(self):
        super().initialise()

        # Handle mock mode
        if self.mock_mode:
            # In mock mode, randomly select a word from the wordlist
            import random

            mock_result = (
                random.choice(self.wordlist) if self.wordlist else "mock_phrase"
            )
            self.blackboard.phrase = mock_result
            self.feedback_message = f"MOCK: Extracted phrase '{mock_result}'"
            print(
                f"🎤 MOCK PHRASE EXTRACTION: '{mock_result}' (from wordlist: {self.wordlist[:3]}...)"
            )
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
                self.feedback_message = (
                    f"Extracted phrase: {self.response.result().phrase}"
                )
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

    def __init__(
        self,
        name: str,
        bb_dest_key: str,
        service_name: str = "target_extraction_service",
        timeout: float = 15.0,
    ):
        super(BtNode_TargetExtraction, self).__init__(
            name, service_name, PhraseExtraction
        )

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="target",
            access=pytree.common.Access.WRITE,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_dest_key),
        )
        self.timeout = timeout
        self.node = None

    def initialise(self):
        super().initialise()

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
                self.feedback_message = (
                    f"Extracted target: {self.response.result().phrase}"
                )
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
    DEPRECATED: use BtNode_GetConfirmationAction. The tk_24_audio package has
    migrated confirmation to a ROS 2 action (`get_confirmation_action`); the
    service-based `get_confirmation_service` will be retired once all task
    trees (GPSR, EGPSR, Receptionist, Restaurant, help-me-carry, serve-breakfast,
    store-groceries) have been migrated.
    """

    def __init__(
        self,
        name: str,
        service_name: str = "get_confirmation_service",
        timeout: float = 15.0,
    ):
        warnings.warn(
            "BtNode_GetConfirmation is deprecated; use BtNode_GetConfirmationAction "
            "(tk_24_audio migrated to action `get_confirmation_action`).",
            DeprecationWarning,
            stacklevel=2,
        )
        super(BtNode_GetConfirmation, self).__init__(
            name, service_name, GetConfirmation
        )
        self.timeout = timeout

    def setup(self, **kwargs):
        super().setup(**kwargs)

        self.logger.debug(f"Setup getting confirmation")

    def initialise(self):
        super().initialise()

        # Handle mock mode via shared input controller
        if self.mock_mode:
            self.feedback_message = "MOCK: Press Enter to confirm"
            print(
                "✅ MOCK GET CONFIRMATION: Press Enter to confirm (will return SUCCESS)"
            )
            return

        request = GetConfirmation.Request()
        request.timeout = self.timeout
        self.response = self.call_service_async(request)
        self.feedback_message = f"Initialized get confirmation"

    def update(self) -> Status:
        # Handle mock mode via shared input controller
        if self.mock_mode:
            return self.wait_for_keypress_in_mock()

        if self.response is None:
            return pytree.common.Status.FAILURE

        self.logger.debug(f"Update get confirmation")
        if self.response.done():
            if self.response.result().status == 0:
                self.feedback_message = (
                    f"Got confirmation: {self.response.result().confirmed}"
                )
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
        super().terminate(new_status)


class BtNode_Listen(ServiceHandler):
    """
    Listens for speech input and stores the recognized message on the blackboard
    for use by other nodes like phrase extraction or target extraction.

    DEPRECATED: use BtNode_ListenAction. The tk_24_audio package has migrated
    Listen to a ROS 2 action (`listen_action`); the service-based
    `listen_service` will be retired once all task trees (GPSR, EGPSR,
    Receptionist, Restaurant, help-me-carry, serve-breakfast, store-groceries)
    have been migrated.
    """

    def __init__(
        self,
        name: str,
        bb_dest_key: str,
        service_name="listen_service",
        timeout: float = 5.0,
    ):
        warnings.warn(
            "BtNode_Listen is deprecated; use BtNode_ListenAction "
            "(tk_24_audio migrated to action `listen_action`).",
            DeprecationWarning,
            stacklevel=2,
        )
        super().__init__(name, service_name, Listen)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="message",
            access=pytree.common.Access.WRITE,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_dest_key),
        )
        self.timeout = timeout

    def initialise(self):
        super().initialise()

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
    """
    Compares interests between two statements for conversation matching.

    This node takes two speech statements from the blackboard and analyzes
    them to find common interests or topics. The result is stored on the
    blackboard for use in conversation or announcement nodes.
    """

    def __init__(
        self,
        name: str,
        bb_source_key1: str,
        bb_source_key2: str,
        bb_dest_key: str,
        service_name="compare_interest_service",
        timeout: float = 5.0,
    ):
        super().__init__(name, service_name, CompareInterest)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="first_statement",
            access=pytree.common.Access.READ,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_source_key1),
        )
        self.blackboard.register_key(
            key="second_statement",
            access=pytree.common.Access.READ,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_source_key2),
        )
        self.blackboard.register_key(
            key="result",
            access=pytree.common.Access.WRITE,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_dest_key),
        )
        self.timeout = timeout

    def initialise(self):
        super().initialise()

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
                self.feedback_message = (
                    f"Compare interest result: {self.response.result().common_interest}"
                )
                return Status.SUCCESS
            else:
                self.feedback_message = f"Compare interest failed with error code {self.response.result().status}: {self.response.result().error_message}"
                return Status.FAILURE
        else:
            self.feedback_message = "Still comparing interest..."
            return Status.RUNNING


class _MockFuture:
    """Stand-in future used in ActionHandler mock paths — always reports done."""

    def done(self):
        return True


class BtNode_GetConfirmationAction(ActionHandler):
    """
    Action-client variant of BtNode_GetConfirmation targeting tk_24_audio's
    `get_confirmation_action` (tinker_audio_msgs/action/GetConfirmation).

    Returns SUCCESS only when the action succeeds AND `result.confirmed` is True
    (matches the legacy service contract).

    NOTE (2026-04): the server's execute_callback runs Kimi classification then
    Qwen multimodal classification **sequentially** — total latency can reach
    ~20 s on top of `timeout`. The feedback_timeout here is set wide enough to
    cover that. When the audio package switches to concurrent LLM cross-check,
    drop `_feedback_timeout_secs` back to ~15.
    """

    def __init__(
        self,
        name: str,
        timeout: float = 15.0,
        action_name: str = "get_confirmation_action",
        wait_for_server_timeout_sec: float = -3.0,
    ):
        super().__init__(
            name,
            GetConfirmationAction,
            action_name,
            key=None,
            wait_for_server_timeout_sec=wait_for_server_timeout_sec,
        )
        self.timeout = timeout
        self._feedback_timeout_secs = max(self.timeout + 5.0, 30.0)

    def send_goal(self):
        if self.mock_mode:
            self.feedback_message = "MOCK: GetConfirmation goal sent"
            self.send_goal_future = _MockFuture()
            return
        goal = GetConfirmationAction.Goal()
        goal.timeout = float(self.timeout)
        self.send_goal_request(goal)
        self.feedback_message = f"sent GetConfirmation goal (timeout={self.timeout}s)"

    def feedback_callback(self, msg):
        feedback = msg.feedback
        self.last_feedback_time = time.time()
        self.feedback_timeout = self._feedback_timeout_secs
        self.action_status = 0
        progress = getattr(feedback, "progress", 0.0)
        status_message = getattr(feedback, "status_message", "")
        partial = getattr(feedback, "partial_transcription", "")
        self.feedback_message = (
            f"confirmation progress={progress:.2f} {status_message}"
            + (f" [partial: '{partial}']" if partial else "")
        )

    def process_result(self):
        if self.result_status != action_msgs.GoalStatus.STATUS_SUCCEEDED:
            err = getattr(
                getattr(self.result_message, "result", None), "error_message", ""
            )
            self.feedback_message = f"GetConfirmation action did not succeed ({self.result_status_string}): {err}"
            return Status.FAILURE
        result = self.result_message.result
        if getattr(result, "confirmed", False):
            self.feedback_message = "Got confirmation: True"
            return Status.SUCCESS
        self.feedback_message = "Got confirmation: False"
        return Status.FAILURE


class BtNode_ListenAction(ActionHandler):
    """
    Action-client variant of BtNode_Listen targeting tk_24_audio's
    `listen_action` (tinker_audio_msgs/action/Listen).

    On success, writes `result.message` to the blackboard at `bb_dest_key`.
    """

    def __init__(
        self,
        name: str,
        bb_dest_key: str,
        timeout: float = 5.0,
        action_name: str = "listen_action",
        wait_for_server_timeout_sec: float = -3.0,
    ):
        super().__init__(
            name,
            ListenAction,
            action_name,
            key=None,
            wait_for_server_timeout_sec=wait_for_server_timeout_sec,
        )
        self.timeout = timeout
        self._feedback_timeout_secs = max(self.timeout + 10.0, 30.0)
        self.bb_dest_key = bb_dest_key
        self.message_blackboard = self.attach_blackboard_client(
            name=f"{self.name}_ListenAction"
        )
        self.message_blackboard.register_key(
            key="message",
            access=pytree.common.Access.WRITE,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_dest_key),
        )

    def send_goal(self):
        if self.mock_mode:
            self.message_blackboard.message = "This is a mock speech input message"
            self.feedback_message = "MOCK: Listen goal sent, wrote mock transcription"
            print(f"👂 MOCK LISTEN (action): '{self.message_blackboard.message}'")
            self.send_goal_future = _MockFuture()
            return
        goal = ListenAction.Goal()
        goal.timeout = float(self.timeout)
        self.send_goal_request(goal)
        self.feedback_message = f"sent Listen goal (timeout={self.timeout}s)"

    def feedback_callback(self, msg):
        feedback = msg.feedback
        self.last_feedback_time = time.time()
        self.feedback_timeout = self._feedback_timeout_secs
        self.action_status = 0
        progress = getattr(feedback, "progress", 0.0)
        status_message = getattr(feedback, "status_message", "")
        partial = getattr(feedback, "partial_transcription", "")
        self.feedback_message = f"listen progress={progress:.2f} {status_message}" + (
            f" [partial: '{partial}']" if partial else ""
        )

    def process_result(self):
        if self.result_status != action_msgs.GoalStatus.STATUS_SUCCEEDED:
            err = getattr(
                getattr(self.result_message, "result", None), "error_message", ""
            )
            self.feedback_message = (
                f"Listen action did not succeed ({self.result_status_string}): {err}"
            )
            return Status.FAILURE
        result = self.result_message.result
        if getattr(result, "status", 0) != 0:
            self.feedback_message = f"Listen failed with code {result.status}: {getattr(result, 'error_message', '')}"
            return Status.FAILURE
        self.message_blackboard.message = result.message
        self.feedback_message = f"Listened: '{result.message}'"
        return Status.SUCCESS


class BtNode_PhraseExtractionAction(ActionHandler):
    """
    Action-client variant of BtNode_PhraseExtraction targeting tk_24_audio's
    `phrase_extraction_action` (tinker_audio_msgs/action/PhraseExtraction).

    The server only calls `goal_handle.succeed()` when status=0 — the Whisper
    + Qwen ASR cross-check agrees on the same wordlist entry. Any other
    status (1 timeout, 2 transcription error, 3 not in wordlist, 4 ASR
    disagreement) calls `goal_handle.abort()`. We therefore map
    `STATUS_SUCCEEDED` → SUCCESS (phrase usable, downstream confirmation can
    be skipped) and everything else → FAILURE (let the outer Retry / Selector
    re-prompt or escalate to a confirmation fallback).

    Feedback schema is `{progress, status_message, partial_transcription}` —
    not the canonical BT `{stage, stage_name, status, delay_limit}`. Same
    override pattern as `BtNode_GetConfirmationAction` / `BtNode_ListenAction`.
    """

    def __init__(
        self,
        name: str,
        wordlist: list,
        bb_dest_key: str,
        timeout: float = 7.0,
        action_name: str = "phrase_extraction_action",
        wait_for_server_timeout_sec: float = -3.0,
    ):
        super().__init__(
            name,
            PhraseExtractionAction,
            action_name,
            key=None,
            wait_for_server_timeout_sec=wait_for_server_timeout_sec,
        )
        self.wordlist = list(wordlist)
        self.timeout = timeout
        # Whisper + Qwen ASR run sequentially after recording; give margin.
        self._feedback_timeout_secs = max(self.timeout + 15.0, 30.0)
        self.bb_dest_key = bb_dest_key
        self._bb = self.attach_blackboard_client(
            name=f"{self.name}_PhraseExtractionAction"
        )
        self._bb.register_key(
            key="phrase",
            access=pytree.common.Access.WRITE,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_dest_key),
        )

    def send_goal(self):
        if self.mock_mode:
            import random

            picked = random.choice(self.wordlist) if self.wordlist else "mock_phrase"
            self._bb.phrase = picked
            self.feedback_message = (
                f"MOCK: picked '{picked}' from {self.wordlist[:3]}..."
            )
            print(f"🎤 MOCK PHRASE EXTRACTION (action): '{picked}'")
            self.send_goal_future = _MockFuture()
            return
        goal = PhraseExtractionAction.Goal()
        goal.timeout = float(self.timeout)
        goal.wordlist = self.wordlist
        self.send_goal_request(goal)
        self.feedback_message = (
            f"sent PhraseExtraction goal (timeout={self.timeout}s, "
            f"|wordlist|={len(self.wordlist)})"
        )

    def feedback_callback(self, msg):
        feedback = msg.feedback
        self.last_feedback_time = time.time()
        self.feedback_timeout = self._feedback_timeout_secs
        self.action_status = 0
        progress = getattr(feedback, "progress", 0.0)
        status_message = getattr(feedback, "status_message", "")
        partial = getattr(feedback, "partial_transcription", "")
        self.feedback_message = f"phrase progress={progress:.2f} {status_message}" + (
            f" [partial: '{partial}']" if partial else ""
        )

    def process_result(self):
        if self.result_status != action_msgs.GoalStatus.STATUS_SUCCEEDED:
            result = getattr(self.result_message, "result", None)
            status = getattr(result, "status", -1)
            err = getattr(result, "error_message", "")
            self.feedback_message = (
                f"PhraseExtraction aborted (action={self.result_status_string}, "
                f"server status={status}): {err}"
            )
            return Status.FAILURE
        phrase = getattr(self.result_message.result, "phrase", "")
        self._bb.phrase = phrase
        self.feedback_message = f"extracted high-confidence phrase: '{phrase}'"
        return Status.SUCCESS
