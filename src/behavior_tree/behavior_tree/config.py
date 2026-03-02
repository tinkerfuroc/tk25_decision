"""
Configuration module for behavior tree mock mode.
Detects whether required ROS2 packages are available and determines mock mode.
Supports fine-grained subsystem-level mock configuration via JSON file.
"""

import os
import json
import importlib.util
from pathlib import Path
from typing import Dict, Optional, Tuple


NODE_MOCK_MODE_MAP = {
    "NO_MOCK": "no_mock",
    "OFF": "no_mock",
    "NONE": "no_mock",
    "FALSE": "no_mock",
    "0": "no_mock",
    "KEYPRESS": "wait_keypress",
    "WAIT_KEYPRESS": "wait_keypress",
    "WAIT": "wait_keypress",
    "MOCK": "wait_keypress",
    "TRUE": "wait_keypress",
    "1": "wait_keypress",
    "TELEOP": "teleop",
    "IMMEDIATE": "immediate",
}


class BehaviorTreeConfig:
    """Configuration manager for behavior tree mock mode."""
    
    _instance = None
    
    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(BehaviorTreeConfig, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance
    
    def __init__(self):
        if self._initialized:
            return
        
        self._initialized = True
        self._dependency_cache = {}
        self._mock_config = None
        self._config_path = None
        self._config_mtime = None
        
        # List of required packages
        # Note: Only check modules that actually exist for each package
        self.required_packages = {
            'tinker_vision_msgs': ['tinker_vision_msgs.srv', 'tinker_vision_msgs.msg', 'tinker_vision_msgs.action'],
            'tinker_arm_msgs': ['tinker_arm_msgs.srv', 'tinker_arm_msgs.action'],
            'tinker_audio_msgs': ['tinker_audio_msgs.srv'],  # Only has services, no messages
            'tinker_nav_msgs': ['tinker_nav_msgs.srv'],  # Only has services
            'control_msgs': ['control_msgs.action'],
            'action_msgs': ['action_msgs.msg'],
            'nav2_msgs': ['nav2_msgs.action'],
        }
        
        # Load mock configuration
        self._load_mock_config()
    
    def _resolve_config_path(self) -> Path:
        """
        Resolve mock config path in a user-friendly order:
        1. `BT_MOCK_CONFIG` env override
        2. Workspace source path discovered from cwd ancestors
        3. Package-local default (`behavior_tree/mock_config.json`)
        """
        env_config_path = os.environ.get('BT_MOCK_CONFIG')
        if env_config_path:
            return Path(env_config_path)

        cwd = Path.cwd().resolve()
        for parent in [cwd] + list(cwd.parents):
            candidate = parent / "src" / "behavior_tree" / "behavior_tree" / "mock_config.json"
            if candidate.exists():
                return candidate

        return Path(__file__).parent / 'mock_config.json'

    def _load_mock_config(self, force: bool = False):
        """Load mock configuration from JSON file."""
        try:
            config_path = self._resolve_config_path()

            if (
                not force
                and self._config_path is not None
                and config_path == self._config_path
                and config_path.exists()
            ):
                mtime = config_path.stat().st_mtime
                if self._config_mtime is not None and mtime == self._config_mtime:
                    return

            if config_path.exists():
                with open(config_path, 'r') as f:
                    self._mock_config = json.load(f)
                self._config_path = config_path
                self._config_mtime = config_path.stat().st_mtime
                print(f"✓ Loaded mock configuration from {config_path}")
            else:
                print(f"⚠ Warning: Mock config not found at {config_path}, using defaults")
                self._mock_config = self._get_default_config()
                self._config_path = config_path
                self._config_mtime = None
        except Exception as e:
            print(f"⚠ Error loading mock config: {e}, using defaults")
            self._mock_config = self._get_default_config()
            self._config_path = None
            self._config_mtime = None

    def _maybe_reload_config(self):
        """Reload config if file changed since last read."""
        self._load_mock_config(force=False)
    
    def _get_default_config(self) -> dict:
        """Return default configuration."""
        return {
            "mock_mode": {
                "enabled": True,
                "auto_detect": True,
                "subsystems": {
                    "vision": {"enabled": True, "nodes": {}},
                    "manipulation": {"enabled": True, "nodes": {}},
                    "navigation": {"enabled": True, "nodes": {}},
                    "audio_input": {"enabled": True, "nodes": {}},
                    "announcement": {"enabled": False, "nodes": {}}
                }
            },
            "keyboard_control": {"enabled": True},
            "mock_keyboard": {
                "start_input_key": "\\",
                "stop_input_key": "/",
                "subsystem_start_keys": {
                    "vision": "p",
                    "manipulation": ";",
                    "navigation": "'",
                    "audio_input": "`",
                    "announcement": "q"
                },
                "success_key": "ENTER"
            },
            "teleop": {
                "params": {}
            },
            "logging": {"print_mock_operations": True, "use_emoji": True}
        }

    def _normalize_node_mode(self, mode_value) -> str:
        """Normalize a configured node mode into internal mode values."""
        if isinstance(mode_value, str):
            key = mode_value.strip().upper()
            return NODE_MOCK_MODE_MAP.get(key, "wait_keypress")
        if isinstance(mode_value, bool):
            return "wait_keypress" if mode_value else "no_mock"
        if isinstance(mode_value, (int, float)):
            return "wait_keypress" if mode_value else "no_mock"
        return "wait_keypress"

    def _find_node_subsystem_entry(self, node_class_name: str) -> Tuple[Optional[str], Optional[dict], object]:
        """
        Find node entry from subsystem config.

        Returns:
            (subsystem_name, subsystem_config, raw_mode_value)
        """
        subsystems = self._mock_config.get("mock_mode", {}).get("subsystems", {})
        for subsystem_name, subsystem_config in subsystems.items():
            nodes_cfg = subsystem_config.get("nodes", {})
            if isinstance(nodes_cfg, dict):
                if node_class_name in nodes_cfg:
                    return subsystem_name, subsystem_config, nodes_cfg[node_class_name]
            elif isinstance(nodes_cfg, list):
                # Backward compatibility: list means mocked via keypress
                if node_class_name in nodes_cfg:
                    return subsystem_name, subsystem_config, "KEYPRESS"
        return None, None, None

    def get_node_subsystem_name(self, node_class_name: str) -> Optional[str]:
        """Get subsystem name that owns this node class."""
        subsystem_name, _, _ = self._find_node_subsystem_entry(node_class_name)
        return subsystem_name
    
    def has_dependency(self, package_name: str) -> bool:
        """
        Check if a package is available for import.
        
        Args:
            package_name: Name of the package to check
            
        Returns:
            True if package is available, False otherwise
        """
        if package_name in self._dependency_cache:
            return self._dependency_cache[package_name]
        
        try:
            # Try to find the package
            if package_name in self.required_packages:
                # Check all submodules - try actual import for better reliability
                for submodule in self.required_packages[package_name]:
                    try:
                        # First try find_spec for quick check
                        spec = importlib.util.find_spec(submodule)
                        if spec is None:
                            self._dependency_cache[package_name] = False
                            return False
                        # Then try actual import to be sure (ROS2 packages need this)
                        __import__(submodule)
                    except (ImportError, ModuleNotFoundError, ValueError):
                        self._dependency_cache[package_name] = False
                        return False
            else:
                spec = importlib.util.find_spec(package_name)
                if spec is None:
                    self._dependency_cache[package_name] = False
                    return False
            
            self._dependency_cache[package_name] = True
            return True
        except (ImportError, ModuleNotFoundError, ValueError):
            self._dependency_cache[package_name] = False
            return False
    
    def is_mock_mode(self) -> bool:
        """
        Determine if mock mode should be enabled globally.
        
        Checks environment variable BT_MOCK_MODE first, then checks JSON config,
        then auto-detects based on missing dependencies if auto_detect is enabled.
        
        Returns:
            True if mock mode should be enabled
        """
        self._maybe_reload_config()

        # Check environment variable first (highest priority)
        env_mock = os.environ.get('BT_MOCK_MODE', '').lower()
        if env_mock in ('true', '1', 'yes'):
            return True
        if env_mock in ('false', '0', 'no'):
            return False
        
        # Check JSON config
        if not self._mock_config.get('mock_mode', {}).get('enabled', True):
            return False
        
        # Auto-detect if enabled
        if self._mock_config.get('mock_mode', {}).get('auto_detect', True):
            for package_name in self.required_packages.keys():
                if not self.has_dependency(package_name):
                    print(f"🔄 Mock mode auto-enabled: {package_name} not found")
                    return True
        
        return False
    
    def is_subsystem_mocked(self, subsystem: str) -> bool:
        """
        Check if a specific subsystem should be mocked.
        
        Args:
            subsystem: One of 'vision', 'manipulation', 'navigation', 'audio_input', 'announcement'
            
        Returns:
            True if the subsystem should be mocked
        """
        # If global mock mode is off, nothing is mocked
        if not self.is_mock_mode():
            return False
        
        # Check subsystem-specific setting
        subsystems = self._mock_config.get('mock_mode', {}).get('subsystems', {})
        return subsystems.get(subsystem, {}).get('enabled', True)
    
    def is_node_mocked(self, node_class_name: str) -> bool:
        """
        Check if a specific node should be mocked based on its class name.
        
        Args:
            node_class_name: Name of the node class (e.g., 'BtNode_Announce')
            
        Returns:
            True if the node should be mocked
        """
        # If global mock mode is off, nothing is mocked
        if not self.is_mock_mode():
            return False
        
        _, subsystem_config, raw_mode = self._find_node_subsystem_entry(node_class_name)
        if subsystem_config is None:
            # Explicit listing is required in the new format
            return False

        if not subsystem_config.get("enabled", True):
            # Subsystem disabled means ignore node mode values
            return False

        return self._normalize_node_mode(raw_mode) != "no_mock"
    
    def should_announce_movement(self, node_class_name: str) -> bool:
        """
        Check if node movements should be announced via TTS in mock mode.
        
        Args:
            node_class_name: Name of the node class
            
        Returns:
            True if movements should be announced
        """
        if not self.is_mock_mode():
            return False
        
        _, subsystem_config, _ = self._find_node_subsystem_entry(node_class_name)
        if subsystem_config is None:
            return False
        if not subsystem_config.get("enabled", True):
            return False
        return subsystem_config.get("announce_movement", False)
    
    def should_use_keyboard_control(self) -> bool:
        """Check if keyboard control should be used in mock mode."""
        return self._mock_config.get('keyboard_control', {}).get('enabled', True)

    def get_node_mock_interaction_mode(self, node_class_name: str) -> str:
        """
        Get mock interaction mode for a specific node.

        Modes:
        - 'teleop': use BtNode_MoveArmTeleop as the mock interaction backend
        - 'wait_keypress': wait for any keypress before succeeding
        - 'immediate': return success immediately in mock mode
        """
        if not self.is_mock_mode():
            return "no_mock"

        _, subsystem_config, raw_mode = self._find_node_subsystem_entry(node_class_name)
        if subsystem_config is None:
            return "no_mock"
        if not subsystem_config.get("enabled", True):
            return "no_mock"

        mode = self._normalize_node_mode(raw_mode)
        if mode == "wait_keypress" and not self.should_use_keyboard_control():
            return "immediate"
        return mode

    def get_mock_teleop_params(self) -> Dict:
        """
        Get constructor kwargs for BtNode_MoveArmTeleop from config.
        """
        self._maybe_reload_config()

        def _normalize_teleop_params(params: Dict) -> Dict:
            if not isinstance(params, dict):
                return {}
            normalized = dict(params)
            alias_map = {
                "linear_velocity": "linear_speed",
                "angular_velocity": "angular_speed",
                "joint_velocity": "joint_speed",
                "frame_id": "command_frame",
            }
            for src_key, dst_key in alias_map.items():
                if src_key in normalized and dst_key not in normalized:
                    normalized[dst_key] = normalized[src_key]
            return normalized

        # Preferred new location
        teleop_cfg = self._mock_config.get("teleop", {})
        params = teleop_cfg.get("params", {})
        # Backward-compatible fallback
        if not isinstance(params, dict):
            interaction = self._mock_config.get("mock_interaction", {})
            teleop_cfg = interaction.get("teleop", {})
            params = teleop_cfg.get("params", {})
        if isinstance(params, dict):
            return _normalize_teleop_params(params)
        return {}

    def get_mock_keyboard_config(self) -> Dict:
        """Get keyboard routing config for mock input controller."""
        self._maybe_reload_config()
        cfg = self._mock_config.get("mock_keyboard", {})
        if not isinstance(cfg, dict):
            return {}
        subsystem_keys = cfg.get("subsystem_start_keys", {})
        if not isinstance(subsystem_keys, dict):
            subsystem_keys = {}
        return {
            "start_input_key": cfg.get("start_input_key", "\\"),
            "stop_input_key": cfg.get("stop_input_key", "/"),
            "subsystem_start_keys": subsystem_keys,
            "success_key": cfg.get("success_key", "ENTER"),
        }
    
    def should_print_mock_operations(self) -> bool:
        """Check if mock operations should be printed."""
        return self._mock_config.get('logging', {}).get('print_mock_operations', True)
    
    def should_use_emoji(self) -> bool:
        """Check if emoji should be used in logging."""
        return self._mock_config.get('logging', {}).get('use_emoji', True)
    
    def print_status(self):
        """Print current configuration status."""
        emoji = self.should_use_emoji()
        border = "=" * 70
        
        print(border)
        print("Behavior Tree Configuration Status")
        print(border)
        
        # Global mock mode status
        global_mock = self.is_mock_mode()
        status_icon = "✓" if emoji else "+"
        print(f"\nGlobal Mock Mode: {status_icon if global_mock else '✗'} {'ENABLED' if global_mock else 'DISABLED'}")
        
        if global_mock:
            print("\nSubsystem Mock Status:")
            subsystems = self._mock_config.get('mock_mode', {}).get('subsystems', {})
            for name, config in subsystems.items():
                enabled = config.get('enabled', False)
                icon = "✓" if (emoji and enabled) else ("✗" if emoji else "-")
                status_text = "MOCKED" if enabled else "REAL"
                print(f"  {icon} {name:15s}: {status_text}")
                if enabled:
                    nodes_cfg = config.get("nodes", {})
                    if isinstance(nodes_cfg, dict):
                        for node_name, mode in nodes_cfg.items():
                            normalized = self._normalize_node_mode(mode)
                            print(f"      - {node_name}: {normalized}")
        
        print(f"\nKeyboard Control: {'✓' if self.should_use_keyboard_control() else '✗'}")
        
        print("\nDependency Availability:")
        for dep_name in self.required_packages.keys():
            available = self.has_dependency(dep_name)
            icon = "✓" if (emoji and available) else ("✗" if emoji else "-")
            status = "Available" if available else "Not Available"
            print(f"  {icon} {dep_name:20s}: {status}")
        
        print(border)
        if global_mock:
            print("\n💡 TIP: You can customize mock behavior in mock_config.json")
            print("   Set BT_MOCK_MODE=false to disable all mocking")
        print(border)


# Global instance
_config = BehaviorTreeConfig()


def get_config() -> BehaviorTreeConfig:
    """Get the global configuration instance."""
    return _config


def is_mock_mode() -> bool:
    """Quick check if global mock mode is enabled."""
    return _config.is_mock_mode()


def is_subsystem_mocked(subsystem: str) -> bool:
    """Check if a specific subsystem is mocked."""
    return _config.is_subsystem_mocked(subsystem)


def is_node_mocked(node_class_name: str) -> bool:
    """Check if a specific node should be mocked."""
    return _config.is_node_mocked(node_class_name)


def should_use_keyboard_control() -> bool:
    """Check if keyboard control should be used in mock mode."""
    return _config.should_use_keyboard_control()


def get_node_mock_interaction_mode(node_class_name: str) -> str:
    """Get interaction mode for a mocked node."""
    return _config.get_node_mock_interaction_mode(node_class_name)


def get_mock_teleop_params() -> Dict:
    """Get BtNode_MoveArmTeleop kwargs configured for mock interaction."""
    return _config.get_mock_teleop_params()


def get_node_subsystem_name(node_class_name: str) -> Optional[str]:
    """Get subsystem name for the given node class."""
    return _config.get_node_subsystem_name(node_class_name)


def get_mock_keyboard_config() -> Dict:
    """Get mock keyboard routing config."""
    return _config.get_mock_keyboard_config()


def should_announce_movement(node_class_name: str) -> bool:
    """Check if node movements should be announced via TTS."""
    return _config.should_announce_movement(node_class_name)


# Global TTS engine instance (reused to avoid segfaults)
_tts_engine = None
_tts_failed = False

def announce_node_action(node_name: str, node_class_name: str):
    """
    Announce node action using pyttsx3 if announce_movement is enabled.
    
    Args:
        node_name: The name of the node instance (e.g., "Move arm to bag position")
        node_class_name: The class name (e.g., "BtNode_MoveArmSingle")
    """
    global _tts_engine, _tts_failed
    
    if not should_announce_movement(node_class_name):
        return
    
    # If TTS failed before, just print
    if _tts_failed:
        print(f"📢 Mock: {node_name}")
        return
    
    try:
        # Initialize engine once and reuse it
        if _tts_engine is None:
            import pyttsx3
            _tts_engine = pyttsx3.init()
            # Set properties for faster, clearer speech
            _tts_engine.setProperty('rate', 175)  # Speed of speech
            _tts_engine.setProperty('volume', 0.9)  # Volume (0.0 to 1.0)
        
        # Announce the action
        message = f"Mock: {node_name}"
        print(f"🔊 Announcing: {message}")
        _tts_engine.say(message)
        _tts_engine.runAndWait()
    except Exception as e:
        # If pyttsx3 fails, disable it and just print
        _tts_failed = True
        _tts_engine = None
        print(f"⚠️ TTS disabled due to error: {e}")
        print(f"📢 Mock: {node_name}")


# Legacy compatibility functions
def has_tinker_vision() -> bool:
    """Check if tinker_vision_msgs is available."""
    return _config.has_dependency('tinker_vision_msgs')


def has_tinker_arm() -> bool:
    """Check if tinker_arm_msgs is available."""
    return _config.has_dependency('tinker_arm_msgs')


def has_tinker_audio() -> bool:
    """Check if tinker_audio_msgs is available."""
    return _config.has_dependency('tinker_audio_msgs')


def has_tinker_nav() -> bool:
    """Check if tinker_nav_msgs is available."""
    return _config.has_dependency('tinker_nav_msgs')


def has_nav2() -> bool:
    """Check if nav2_msgs is available."""
    return _config.has_dependency('nav2_msgs')


def has_control_msgs() -> bool:
    """Check if control_msgs is available."""
    return _config.has_dependency('control_msgs')


def has_action_msgs() -> bool:
    """Check if action_msgs is available."""
    return _config.has_dependency('action_msgs')
