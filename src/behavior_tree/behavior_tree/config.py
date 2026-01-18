"""
Configuration module for behavior_tree package.
This module handles optional dependencies and mock mode configuration.
"""
import os
import sys
from typing import Dict, Any


class BehaviorTreeConfig:
    """
    Central configuration for behavior tree package.
    Manages mock mode and dependency availability.
    """
    
    def __init__(self):
        # Check if we should use mock mode from environment variable
        self._mock_mode = os.environ.get('BT_MOCK_MODE', 'false').lower() == 'true'
        
        # Track which dependencies are available
        self._available_deps = {
            'tinker_vision_msgs': False,
            'tinker_arm_msgs': False,
            'tinker_audio_msgs': False,
            'tinker_nav_msgs': False,
            'nav2_msgs': False,
            'control_msgs': False,
            'action_msgs': False,
        }
        
        # Try to import each dependency
        self._check_dependencies()
        
        # If mock mode is enabled, pretend nothing is available
        if self._mock_mode:
            for key in self._available_deps:
                self._available_deps[key] = False
    
    def _check_dependencies(self):
        """Check which dependencies are available."""
        # Check tinker_vision_msgs
        try:
            import tinker_vision_msgs.srv
            self._available_deps['tinker_vision_msgs'] = True
        except ImportError:
            pass
        
        # Check tinker_arm_msgs
        try:
            import tinker_arm_msgs.srv
            self._available_deps['tinker_arm_msgs'] = True
        except ImportError:
            pass
        
        # Check tinker_audio_msgs
        try:
            import tinker_audio_msgs.srv
            self._available_deps['tinker_audio_msgs'] = True
        except ImportError:
            pass
        
        # Check tinker_nav_msgs
        try:
            import tinker_nav_msgs.srv
            self._available_deps['tinker_nav_msgs'] = True
        except ImportError:
            pass
        
        # Check nav2_msgs
        try:
            import nav2_msgs.action
            self._available_deps['nav2_msgs'] = True
        except ImportError:
            pass
        
        # Check control_msgs
        try:
            import control_msgs.action
            self._available_deps['control_msgs'] = True
        except ImportError:
            pass
        
        # Check action_msgs
        try:
            import action_msgs.msg
            self._available_deps['action_msgs'] = True
        except ImportError:
            pass
    
    @property
    def mock_mode(self) -> bool:
        """Whether mock mode is enabled."""
        return self._mock_mode
    
    @mock_mode.setter
    def mock_mode(self, value: bool):
        """Enable or disable mock mode."""
        self._mock_mode = value
        if value:
            # Disable all dependencies in mock mode
            for key in self._available_deps:
                self._available_deps[key] = False
    
    def has_dependency(self, dep_name: str) -> bool:
        """Check if a dependency is available."""
        return self._available_deps.get(dep_name, False)
    
    def get_available_dependencies(self) -> Dict[str, bool]:
        """Get dictionary of all dependency availability."""
        return self._available_deps.copy()
    
    def print_status(self):
        """Print current configuration status."""
        print("=" * 60)
        print("Behavior Tree Configuration Status")
        print("=" * 60)
        print(f"Mock Mode: {self._mock_mode}")
        print("\nDependency Availability:")
        for dep, available in self._available_deps.items():
            status = "✓ Available" if available else "✗ Not Available"
            print(f"  {dep}: {status}")
        print("=" * 60)
        if self._mock_mode or not all(self._available_deps.values()):
            print("\nRunning in MOCK MODE - using keyboard press substitutions")
            print("Set environment variable: export BT_MOCK_MODE=false to disable")
        print("=" * 60)


# Global configuration instance
_config = BehaviorTreeConfig()


def get_config() -> BehaviorTreeConfig:
    """Get the global configuration instance."""
    return _config


def is_mock_mode() -> bool:
    """Quick check if mock mode is enabled."""
    return _config.mock_mode


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
