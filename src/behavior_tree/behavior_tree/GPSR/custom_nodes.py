import asyncio
from pytree.nodes.service_handler import ServiceHandler
from pytree.common import Status, Access
from pytree.nodes.behaviour import Behaviour
from pytree.common import Status, Access
import openai
import json
import time
from .config import OPENAI_API_KEY, OPENAI_MODEL, OPENAI_TEMPERATURE, OPENAI_MAX_TOKENS

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
                 bb_info: str,
                 bb_state: str,
                 bb_next_action: str,
                 service_name: str = "decide_next_action"):
        super(BtNode_DecideNextAction, self).__init__(name, service_name)
        self.bb_command = bb_command
        self.bb_action_list = bb_action_list
        self.bb_info = bb_info
        self.bb_state = bb_state
        self.bb_next_action = bb_next_action
        self.bb_client = None
        self.llm_future = None
        # Initialize OpenAI client
        self.client = openai.OpenAI(api_key=OPENAI_API_KEY, base_url="https://openrouter.ai/api/v1")

    def setup(self, **kwargs):
        ServiceHandler.setup(self, **kwargs)
        self.bb_client = self.attach_blackboard_client(name="Decision ReadWrite")
        self.bb_client.register_key(self.bb_command, access=Access.READ)
        self.bb_client.register_key(self.bb_action_list, access=Access.READ)
        self.bb_client.register_key(self.bb_info, access=Access.READ)
        self.bb_client.register_key(self.bb_state, access=Access.READ)
        self.bb_client.register_key(self.bb_next_action, access=Access.WRITE)
        self.logger.debug(f"Setup Decision Node")

    def initialise(self) -> None:
        self.logger.debug("Initialising Decision Node")
        try:
            command = self.bb_client.get(self.bb_command)
            action_list = self.bb_client.get(self.bb_action_list)
            info = self.bb_client.get(self.bb_info)
            state = self.bb_client.get(self.bb_state)
        except Exception as e:
            self.feedback_message = f"Missing blackboard input: {e}"
            raise e

        prompt = (
            f"COMMAND:\n{command}\n\n"
            f"ACTION:\n{action_list}\n\n"
            f"INFO:\n{info}\n\n"
            f"STATE:\n{state}\n\n"
            f"请根据上述内容决定下一步应该调用哪个动作。返回一个JSON格式的响应,包含以下字段:\n"
            f"- action: 动作名称 (string)\n"
            f"- parameters: 动作参数 (object)\n"
            f"例如:\n"
            f'{{"action": "goto", "parameters": {{"target": "bedroom"}}}}\n'
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
                if not all(key in action_data for key in ['action', 'parameters', 'reason']):
                    raise ValueError("Missing required fields in LLM response")
                
                # Format action string with parameters
                action_str = action_data['action']
                if action_data['parameters']:
                    params_str = ', '.join(f"{k}={v}" for k, v in action_data['parameters'].items())
                    action_str = f"{action_str}({params_str})"
                
                self.bb_client.set(self.bb_next_action, action_str)
                self.feedback_message = f"Next action decided: {action_str} (Reason: {action_data['reason']})"
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
                     "content": """ 你现在是一个机器人的决策层,会根据所收到的自然语言指令,自己能完成的动作,一些先验知识以及目前任务已完成状况判断下一个行动
                                    能完成的动作列表:
                                    1. scanfor(object_name): 用顶上相机扫描寻找指定物体
                                        输入: object_name - 要寻找的物体名称
                                        功能: 扫描环境寻找指定物体,将位置记录到blackboard
                                    
                                    2. findobj(object_name, category): 用机械臂腕上相机在指定类别中寻找物体
                                        输入: object_name - 要寻找的物体名称
                                             category - 物体类别
                                        功能: 在指定类别中寻找特定物体，用于在调用grasp前判断抓取的物体位置
                                    
                                    3. announce(message): 语音播报信息
                                        输入: message - 要播报的信息
                                        功能: 通过语音播报指定信息
                                    
                                    4. goto(location): 移动到指定位置
                                        输入: location - 目标位置
                                        功能: 控制机器人移动到指定位置
                                    
                                    6. grasp(target): 抓取物体
                                        输入: target - 要抓取的目标物体
                                        功能: 控制机械臂抓取指定物体
                                    
                                    7. track(person): 跟踪人物
                                        输入: person - 要跟踪的人物
                                        功能: 持续跟踪指定人物的位置
                                    
                                    8. feature(target): 提取特征
                                        输入: target - 要提取特征的目标
                                        功能: 提取目标的视觉特征
                                    
                                    9. seat(persons): 座位推荐
                                        输入: persons - 人员列表
                                        功能: 根据当前人员情况推荐座位
                                    
                                    10. match(features): 特征匹配
                                        输入: features - 要匹配的特征
                                        功能: 将特征与已知特征进行匹配
                                    
                                    11. phrase(wordlist): 语音短语提取
                                        输入: wordlist - 关键词列表
                                        功能: 从语音中提取包含关键词的短语
                                    
                                    12. confirm(): 获取确认
                                        输入: 无
                                        功能: 等待用户确认
                                    
                                    13. calc_grasp(target_point): 计算抓取姿态
                                        输入: target_point - 目标点
                                        功能: 计算抓取目标的最佳姿态

                                    先验知识:
                                        厨房,客厅,厕所的位置:kitchen,living room,toilet
                                        可乐,厕纸的位置:cola_pos,toilet_paper_pos
                                        客厅桌子的位置:table_pos
                                    
                                    例子1:
                                    自然语言指令:到厨房拿可乐并放到客厅里
                                    当前任务完成状态:已经走到柜子面前
                                    你作为决策器需要根据以上资料判断下一步行动,在这个例子中你下一步应该调用scanfor(cola)来找到可乐的位置,以便下一步决策时调用grasp(cola)
                                    
                                    例子2:
                                    自然语言指令:到客厅找到厕纸,送到厕所
                                    当前完成状态:已经抓取厕纸
                                    你作为决策器需要根据以上资料判断下一步行动,在这个例子中你下一步应该调用goto(toilet)来移动到厕所,以便下一步决策时调用putdown(toilet_paper)
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

    def __init__(self, name: str, completed_action: str, bb_state_key: str):
        super(BtNode_UpdateState, self).__init__(name)
        self.completed_action = completed_action
        self.bb_state_key = bb_state_key
        self.bb_client = None

    def setup(self, **kwargs):
        self.bb_client = self.attach_blackboard_client(name="StateUpdater")
        self.bb_client.register_key(self.bb_state_key, access=Access.READ_WRITE)
        self.logger.debug(f"Setup UpdateState node for action: {self.completed_action}")

    def initialise(self):
        self.feedback_message = f"Updating state with action: {self.completed_action}"

    def update(self):
        try:
            current_state = self.bb_client.get(self.bb_state_key)
            if not current_state:
                current_state = ""
            updated_state = current_state.strip() + f"\n已完成动作: {self.completed_action}"
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
