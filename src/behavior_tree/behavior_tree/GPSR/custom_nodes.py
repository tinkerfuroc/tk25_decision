import asyncio
from behavior_tree.TemplateNodes.BaseBehaviors import ServiceHandler
from py_trees.common import Status, Access
from py_trees.behaviour import Behaviour
from py_trees.common import Status, Access
from py_trees.blackboard import Blackboard
import openai
import json
import time
from .config import OPENAI_API_KEY, OPENAI_MODEL, OPENAI_TEMPERATURE, OPENAI_MAX_TOKENS

from behavior_tree.messages import QuestionAnswer, Listen

from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction

# Initialize OpenAI API
openai.api_key = OPENAI_API_KEY

class BtNode_DecideNextAction(ServiceHandler):
    """
    Node for making a high-level decision by calling a language model asynchronously.
    Expects a JSON formatted response from LLM containing action details.
    """

    def __init__(self,
                 name: str,
                 bb_command: str,
                 bb_action_list: str,
                 bb_state: str,
                 bb_next_action: str,
                 bb_params: str,
                 service_name: str = "decide_next_action"):
        super(BtNode_DecideNextAction, self).__init__(name, service_name)
        self.bb_command = bb_command
        self.bb_action_list = bb_action_list
        self.bb_state = bb_state
        self.bb_next_action = bb_next_action
        self.bb_params = bb_params
        self.bb_client = None
        self.llm_future = None
        # Initialize OpenAI client
        self.client = openai.OpenAI(api_key=OPENAI_API_KEY, base_url="https://openrouter.ai/api/v1")

    def setup(self, **kwargs):
        ServiceHandler.setup(self, **kwargs)
        self.bb_client = self.attach_blackboard_client(name="Decision ReadWrite")
        self.bb_client.register_key(self.bb_command, access=Access.READ)
        self.bb_client.register_key(self.bb_action_list, access=Access.READ)
        self.bb_client.register_key(self.bb_params, access=Access.READ)
        self.bb_client.register_key(self.bb_state, access=Access.READ)
        self.bb_client.register_key(self.bb_next_action, access=Access.WRITE)
        self.logger.debug(f"Setup Decision Node")

    def initialise(self) -> None:
        self.logger.debug("Initialising Decision Node")
        try:
            command = self.bb_client.get(self.bb_command)
            action_list = self.bb_client.get(self.bb_action_list)
            state = self.bb_client.get(self.bb_state)
        except Exception as e:
            self.feedback_message = f"Missing blackboard input: {e}"
            raise e

        ''' COMMAND:
        Robot please look for a person raising their right arm in the living room and answer a question 
        question 
        Q: What country holds the record for the most gold medals at the Winter Olympics? 
        A: Canada does! With 14 Golds at the 2010 Vancouver Winter Olympics.
        
        ACTION:
        
        '''
        prompt = (
            f"COMMAND:\n{command}\n\n"
            f"ACTION:\n{action_list}\n\n"
            f"STATE:\n{state}\n\n"
            f"请根据上述内容决定下一步应该调用哪个动作。返回一个JSON格式的响应,包含以下字段:\n"
            f"- action: 动作名称 (string)\n"
            f"- parameters: 动作参数 (object)\n"
            f"例如:\n"
            f'{{"action": "goto", "parameters": "bedroom"}}\n'
            f"确保返回的是合法的JSON格式，记住parameters的数量和类型必须对应ACTION中的参数和类型。"
        )

        # 调用异步 LLM 请求
        self.llm_future = asyncio.ensure_future(self.call_llm_async(prompt))
        self.feedback_message = "LLM called asynchronously, waiting for response..."

    def update(self):
        self.logger.debug("Updating Decision Node")

        if self.llm_future is None:
            self.feedback_message = "LLM future not initialized"
            return Status.FAILURE

        if not self.llm_future.done():
            self.feedback_message = "LLM still thinking..."
            return Status.RUNNING

        try:
            result_text = self.llm_future.result().strip()
            # Parse JSON response
            try:
                action_data = json.loads(result_text)
                # Validate required fields
                if not all(key in action_data for key in ['action', 'parameters']):
                    raise ValueError("Missing required fields in LLM response")
                
                # Format action string with parameters
                action_str = action_data['action']
                params_str = action_data['parameters']
                
                self.bb_client.set(self.bb_next_action, action_str)
                self.bb_client.set(self.bb_params, params_str)
                self.feedback_message = f"Next action decided: {action_str} (Params: {params_str})"
                return Status.SUCCESS
            except json.JSONDecodeError as e:
                self.feedback_message = f"Failed to parse LLM response as JSON: {e}"
                return Status.FAILURE
            except ValueError as e:
                self.feedback_message = f"Invalid LLM response format: {e}"
                return Status.FAILURE
        except Exception as e:
            self.feedback_message = f"LLM call failed: {e}"
            return Status.FAILURE

    async def call_llm_async(self, prompt: str) -> str:
        try:
            start_time = time.time_ns()
            response = await self.client.chat.completions.create(
                model=OPENAI_MODEL,
                messages=[
                    {"role": "system", 
                     "content": """ 你现在是一个机器人的决策层,会根据所收到的自然语言指令,自己能完成的动作,一些先验知识以及目前任务已完成状况判断下一个行动及其参数
                                    能完成的动作列表:
                                    1. qa(): 用户问问题，我回答
                                        输入: 无
                                        功能: 先提醒用户问问题，然后听问题，最后回答该问题
                                        输出: ["qa", ""]
                                    
                                    2. announce(message): 语音播报信息
                                        输入: message - 要播报的信息
                                        功能: 通过语音播报指定信息
                                        输出: ["announce", message]
                                    
                                    3. goto(location): 移动到指定位置
                                        输入: location - 目标位置
                                        功能: 控制机器人移动到指定位置
                                        输出: ["goto", location]
                                    
                                    4. grasp(target): 抓取物体
                                        输入: target - 要抓取的目标物体
                                        功能: 控制机械臂抓取指定物体
                                        输出: ["grasp", target]

                                    先验知识:
                                        location: bed, dresser, desk, dining table, storage box, wine rack, sofa, side table, TV cabinet, storage table, sink, dishwasher, bedroom, dining room, living room, kitchen
                                        target: chip, biscuit, bread, sprite, cola, water, dishsoap, handwash, shampoo, cookie, lays
                                    
                                    例子1:
                                    自然语言指令:到厨房拿可乐并放到客厅里
                                    当前任务完成状态:已经走到柜子面前
                                    你作为决策器需要根据以上资料判断下一步行动,在这个例子中你下一步应该调用grasp(cola)
                                    故应该输出
                                    {
                                        "action": "grasp",
                                        "parameters": "cola"
                                    }
                                    
                                    例子2:
                                    自然语言指令:到客厅找到洗发水,送到厨房
                                    当前完成状态:已经抓取洗发水
                                    你作为决策器需要根据以上资料判断下一步行动,在这个例子中你下一步应该调用goto(kitchen)来移动到厨房
                                    故应该输出
                                    {
                                        "action": "goto",
                                        "parameters": "kitchen"
                                    }
                                """
                    },
                    {"role": "user", "content": prompt}
                ],
                temperature=OPENAI_TEMPERATURE,
                max_tokens=OPENAI_MAX_TOKENS,
                response_format={"type": "json_object"}  # Enforce JSON response format
            )
            return response.choices[0].message.content
        except Exception as e:
            self.logger.error(f"LLM async call failed: {e}")
            raise e

class BtNode_CheckIfMyTurn(Behaviour):
    """
    Guard Node: check if next_action on blackboard is this node's responsibility.
    Return FAILURE if it is this node's turn (so it can proceed),
    Return SUCCESS otherwise (skip this branch).
    """

    def __init__(self, name: str, responsible_action_keyword: str, bb_next_action: str):
        super(BtNode_CheckIfMyTurn, self).__init__(name)
        self.responsible_action_keyword = responsible_action_keyword
        self.bb_next_action = bb_next_action
        self.bb_client = None

    def setup(self, **kwargs):
        self.bb_client = self.attach_blackboard_client(name="CheckIfMyTurn")
        self.bb_client.register_key(self.bb_next_action, access=Access.READ)
        self.logger.debug(f"Setup guard for: {self.responsible_action_keyword}")

    def initialise(self):
        self.feedback_message = f"Checking if next_action matches {self.responsible_action_keyword}"

    def update(self):
        try:
            next_action = self.bb_client.get(self.bb_next_action)
            if next_action and self.responsible_action_keyword in next_action:
                self.feedback_message = f"It is my turn: {next_action}"
                return Status.FAILURE
            else:
                self.feedback_message = f"Not my turn: {next_action}"
                return Status.SUCCESS
        except Exception as e:
            self.feedback_message = f"Error reading blackboard: {e}"
            return Status.FAILURE


class BtNode_UpdateState(Behaviour):
    """
    Node to update the blackboard state after completing an action.
    Typically used after a functional node finishes its task.
    """

    def __init__(self, name: str, bb_params: str, bb_state_key: str):
        super(BtNode_UpdateState, self).__init__(name)
        self.name = name
        self.bb_params = bb_params
        self.bb_state_key = bb_state_key
        self.bb_client = None

    def setup(self, **kwargs):
        self.bb_client = self.attach_blackboard_client(name="StateUpdater")
        self.bb_client.register_key(self.bb_state_key, access=Access.READ_WRITE)
        self.bb_client.register_key(self.bb_params, access=Access.READ)
        self.logger.debug(f"Setup UpdateState node for action: {self.bb_params}")

    def initialise(self):
        self.feedback_message = f"Updating state with action: {self.bb_params}"

    def update(self):
        try:
            current_state = self.bb_client.get(self.bb_state_key)
            if not current_state:
                current_state = ""
            updated_state = ""
            if self.name == "update_after_qa":
                updated_state = f"answered"
            elif self.name == "update_after_announce":
                updated_state = f"announced {self.bb_params}"
            elif self.name == "update_after_goto":
                updated_state = f"arrived {self.bb_params}"
            elif self.name == "update_after_grasp":
                updated_state = f"grasped {self.bb_params}"
            self.bb_client.set(self.bb_state_key, updated_state.strip())
            self.feedback_message = "State updated"
            return Status.SUCCESS
        except Exception as e:
            self.feedback_message = f"Failed to update state: {e}"
            return Status.FAILURE

class BtNode_CheckIfCompleted(Behaviour):
    """
    Guard Node: check if next_action == 'completed'.
    If completed, return FAILURE to stop further execution.
    """

    def __init__(self, name: str, bb_next_action_key: str):
        super(BtNode_CheckIfCompleted, self).__init__(name)
        self.bb_next_action_key = bb_next_action_key
        self.bb_client = None

    def setup(self, **kwargs):
        self.bb_client = self.attach_blackboard_client(name="CompletionChecker")
        self.bb_client.register_key(self.bb_next_action_key, access=Access.READ)
        self.logger.debug(f"Setup completion checker for {self.bb_next_action_key}")

    def initialise(self):
        self.feedback_message = "Checking if task is completed"

    def update(self):
        try:
            next_action = self.bb_client.get(self.bb_next_action_key)
            if next_action == "completed":
                self.feedback_message = "Task completed — halting execution"
                return Status.FAILURE
            else:
                self.feedback_message = f"Task not completed yet: next_action = {next_action}"
                return Status.SUCCESS
        except Exception as e:
            self.feedback_message = f"Error reading blackboard: {e}"
            return Status.FAILURE

class BtNode_QA(ServiceHandler):
    def __init__(self, name: str, bb_key_dest: str, timeout: float = 7.0, service_name = "question_answer_service"):
        super().__init__(name, service_name, QuestionAnswer)
        self.timeout = timeout
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.key = bb_key_dest
        self.blackboard.register_key(
            key="answer",
            access=Access.WRITE,
            remap_to=Blackboard.absolute_name("/", bb_key_dest)
        )
    
    def initialise(self):
        request = QuestionAnswer.Request()
        request.timeout = self.timeout
        self.response = self.client.call_async(request)
        self.feedback_message = f"Initialized QnA, timeout of {self.timeout} seconds"
    
    def update(self):
        self.logger.debug(f"Updated QnA")
        if self.response.done():
            result : QuestionAnswer.Response = self.response.result()
            if result.status == 0:
                self.blackboard.answer = result.answer
                self.feedback_message = f"Answer: {result.answer}"
                return Status.SUCCESS
            else:
                self.feedback_message = f"QnA failed with error code {result.status}: {result.error_message}"
                return Status.FAILURE
        else:
            self.feedback_message = "Still running QnA..."
            return Status.RUNNING


# write PoseStamped to a blackboard location
class BtNode_WritePose(Behaviour):
    def __init__(self,
                 name: str,
                 bb_key_params: str,
                 bb_key_dest: str):
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="param",
            access=Access.WRITE,
            remap_to=Blackboard.absolute_name("/", bb_key_params)
        )
        self.blackboard.register_key(
            key="pose",
            access=Access.WRITE,
            remap_to=Blackboard.absolute_name("/", bb_key_dest)
        )
        # TODO: declare dictionary
    
    def initialise(self) -> None:
        return super().initialise()
    
    def update(self) -> Status:
        # 读出param，对照存入PoseStamped到pose位置
        return super().update()


class BtNode_WriteVisionPrompt(Behaviour):
    def __init__(self,
                 name: str,
                 bb_key_params: str,
                 bb_key_dest: str):
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="param",
            access=Access.WRITE,
            remap_to=Blackboard.absolute_name("/", bb_key_params)
        )
        self.blackboard.register_key(
            key="prompt",
            access=Access.WRITE,
            remap_to=Blackboard.absolute_name("/", bb_key_dest)
        )
    
    def initialise(self) -> None:
        self.descriptions = {"chip": "blue and pink oreo box",
                "biscuit": "yellow chips can",
                "lays": "red chips can",
                "cookie": "black and green cookie box",
                "bread": "white bread",
                "sprite": "green sprite bottle",
                "cola": "black cola bottle",
                "orange juice": "orange bottle",
                "water": "clear water bottle",
                "dishsoap": "yellow and blue bottle",
                "handwash": "white handwash bottle",
                "shampoo": "blue shampoo bottle",
                "cereal bowl": "blue bowl"}
        return super().initialise()
    
    def update(self) -> Status:
        # 读出param，对照存入PoseStamped到pose位置
        if self.blackboard.param.lower() in self.descriptions.keys():
            self.blackboard.prompt = self.descriptions[self.param.lower()]
            return Status.SUCCESS
        self.feedback_message = f"'{self.blackboard.param.lower()}' is not in list of prompts!"
        self.blackboard.prompt = self.blackboard.param.lower()
        return Status.SUCCESS


class BtNode_GetCommand(ServiceHandler):
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
            access=Access.WRITE,
            remap_to=Blackboard.absolute_name("/", bb_dest_key)
        )
        self.timeout = timeout
    
    def initialise(self):
        request = Listen.Request()
        request.timeout = self.timeout
        self.response = self.client.call_async(request)
        self.feedback_message = f"Initialized GetCommand"
    
    def update(self):
        self.logger.debug(f"Update get command")
        if self.response.done():
            if self.response.result().status == 0:
                self.feedback_message = "got command"
                return Status.SUCCESS
            else:
                self.feedback_message = f"Get Command failed with error code {self.response.result().status}: {self.response.result().error_message}"
                return Status.FAILURE
        else:
            self.feedback_message = "Still getting command..."
            return Status.RUNNING
