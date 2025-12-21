"""
配置文件加载器
支持YAML格式的配置文件
"""
import yaml
import os
from typing import Dict, Any, Optional


class ConfigLoader:
    """配置加载器"""
    
    def __init__(self, config_path: str = "config.yaml"):
        """
        初始化配置加载器
        
        Args:
            config_path: 配置文件路径
        """
        self.config_path = config_path
        self.config: Dict[str, Any] = {}
        self.load_config()
    
    def load_config(self) -> Dict[str, Any]:
        """加载配置文件"""
        if not os.path.exists(self.config_path):
            print(f"配置文件不存在: {self.config_path}，使用默认配置")
            self.config = self.get_default_config()
            return self.config
        
        try:
            with open(self.config_path, 'r', encoding='utf-8') as f:
                self.config = yaml.safe_load(f) or {}
            print(f"配置文件加载成功: {self.config_path}")
        except Exception as e:
            print(f"配置文件加载失败: {e}，使用默认配置")
            self.config = self.get_default_config()
        
        return self.config
    
    def get_default_config(self) -> Dict[str, Any]:
        """获取默认配置"""
        return {
            'camera': {
                'index': 0,
                'width': 640,
                'height': 480,
                'fps': 30
            },
            'detection': {
                'threshold': 48,
                'min_area': 100,
                'max_line_offset': 150
            },
            'robot': {
                'connection_type': 'serial',
                'port': 'COM3',
                'baudrate': 115200
            },
            'motion': {
                'max_speed': 100,
                'base_speed': 50,
                'turn_speed': 30
            },
            'navigation': {
                'update_rate': 30,
                'frame_skip': 1
            },
            'display': {
                'show_window': True,
                'show_mask': True
            }
        }
    
    def get(self, key: str, default: Any = None) -> Any:
        """
        获取配置值（支持点号分隔的嵌套键）
        
        Args:
            key: 配置键，如 'camera.index' 或 'detection.threshold'
            default: 默认值
            
        Returns:
            配置值
        """
        keys = key.split('.')
        value = self.config
        
        for k in keys:
            if isinstance(value, dict) and k in value:
                value = value[k]
            else:
                return default
        
        return value
    
    def set(self, key: str, value: Any):
        """
        设置配置值（支持点号分隔的嵌套键）
        
        Args:
            key: 配置键
            value: 配置值
        """
        keys = key.split('.')
        config = self.config
        
        for k in keys[:-1]:
            if k not in config:
                config[k] = {}
            config = config[k]
        
        config[keys[-1]] = value
    
    def save_config(self, path: Optional[str] = None):
        """
        保存配置到文件
        
        Args:
            path: 保存路径，如果为None则使用原路径
        """
        save_path = path or self.config_path
        
        try:
            with open(save_path, 'w', encoding='utf-8') as f:
                yaml.dump(self.config, f, default_flow_style=False, 
                         allow_unicode=True, sort_keys=False)
            print(f"配置已保存到: {save_path}")
        except Exception as e:
            print(f"配置保存失败: {e}")

