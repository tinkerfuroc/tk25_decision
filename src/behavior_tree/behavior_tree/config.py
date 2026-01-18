"""
Configuration module for behavior tree mock mode.
Detects whether required ROS2 packages are available and determines mock mode.
Supports fine-grained subsystem-level mock configuration via JSON file.
"""

import os
import json
import importlib.util
from pathlib import Path
from typing import Dict, Optional


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
        
        # List of required packages
        self.required_packages = {
            'tinker_vision_msgs': ['tinker_vision_msgs.srv', 'tinker_vision_msgs.msg'],
            'tinker_arm_msgs': ['tinker_arm_msgs.srv', 'tinker_arm_msgs.msg'],
            'tinker_audio_msgs': ['tinker_audio_msgs.srv', 'tinker_audio_msgs.msg'],
            'tinker_nav_msgs': ['tinker_nav_msgs.srv', 'tinker_nav_msgs.msg'],
            'control_msgs': ['control_msgs.action'],
            'action_msgs': ['action_msgs.msg'],
            'nav2_msgs': ['nav2_msgs.action'],
        }
        
        # Load mock configuration
        self._load_mock_config()
    
    def _load_mock_config(self):
        """Load mock configuration from JSON file."""
        try:
            # Find the config file relative to this module
            config_path = Path(__file__).parent / 'mock_config.json'
            
            # Check for user override in environment
            env_config_path = os.environ.get('BT_MOCK_CONFIG')
            if env_config_path:
                config_path = Path(env_config_path)
            
            if config_path.exists():
                with open(config_path, 'r') as f:
                    self._mock_config = json.load(f)
                print(f"✓ Loaded mock configuration from {config_path}")
            else:
                print(f"⚠ Warning: Mock config not found at {config_path}, using defaults")
                self._mock_config = self._get_default_config()
        except Exception as e:
            print(f"⚠ Error loading mock config: {e}, using defaults")
            self._mock_config = self._get_default_config()
    
    def _get_default_config(self) -> dict:
        """Return default configuration."""
        return {
            "mock_mode": {
                "enabled": True,
                "auto_detect": True,
                "subsystems": {
                    "vision": {"enabled": True},
                    "manipulation": {"enabled": True},
                    "navigation": {"enabled": True},
                    "audio_input": {"enabled": True},
                    "announcement": {"enabled": False}
                }
            },
            "keyboard_control": {"enabled": True},
            "logging": {"print_mock_operations": True, "use_emoji": True}
        }
    
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
                # Check all submodules
                for submodule in self.required_packages[package_name]:
                    spec = importlib.util.find_spec(submodule)
                    if spec is None:
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
        
        # Check each subsystem to see if this node belongs to it
        subsystems = self._mock_config.get('mock_mode', {}).get('subsystems', {})
        for subsystem_name, subsystem_config in subsystems.items():
            nodes = subsystem_config.get('nodes', [])
            if node_class_name in nodes:
                return subsystem_config.get('enabled', True)
        
        # Default: if not specified, follow global mock mode
        return True
    
    def should_use_keyboard_control(self) -> bool:
        """Check if keyboard control should be used in mock mode."""
        return self._mock_config.get('keyboard_control', {}).get('enabled', True)
    
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
