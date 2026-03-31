#!/usr/bin/env python3
"""
Map Save Service Node
统一地图保存服务节点，支持同时保存 PGM 和 PCD
"""

import os
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from datetime import datetime
from interface.srv import SaveAllMaps, SaveMaps
from nav2_msgs.srv import SaveMap


class MapSaveServiceNode(Node):
    def __init__(self):
        super().__init__('map_save_service_node')
        
        # 基础路径
        self.base_path = '/home/qiaowen/rpp_ws/src/robot_localization/fastlio2_ros2/map'
        self.pgm_base = os.path.join(self.base_path, 'pgm')
        self.pcd_base = os.path.join(self.base_path, 'pcd')
        
        # 创建服务
        self.srv = self.create_service(
            SaveAllMaps,
            '/save_all_maps',
            self.save_all_maps_callback
        )

        # 回调组：允许服务回调中同步等待下游服务响应
        self.client_cb_group = ReentrantCallbackGroup()

        # 创建客户端
        # nav2_map_server 在不同 launch 中常见两种命名，这里都兼容
        self.pgm_service_names = [
            '/map_saver_server/save_map',
            '/map_saver/save_map',
        ]
        self.pgm_clients = {
            service_name: self.create_client(
                SaveMap,
                service_name,
                callback_group=self.client_cb_group
            )
            for service_name in self.pgm_service_names
        }
        self.pcd_service_name = '/pgo/save_maps'
        self.pcd_client = self.create_client(
            SaveMaps,
            self.pcd_service_name,
            callback_group=self.client_cb_group
        )

        # 超时控制
        self.pgm_wait_timeout = 2.0
        self.pcd_wait_timeout = 2.0
        self.pgm_call_timeout = 15.0
        self.pcd_call_timeout = 120.0

        self.get_logger().info('Map Save Service Node started')
        self.get_logger().info(f'PGM service candidates: {self.pgm_service_names}')
        self.get_logger().info(f'PCD service: {self.pcd_service_name}')
        self.get_logger().info(f'PGM base path: {self.pgm_base}')
        self.get_logger().info(f'PCD base path: {self.pcd_base}')

    def _find_available_pgm_client(self):
        """查找可用的 PGM 保存服务客户端"""
        for service_name in self.pgm_service_names:
            client = self.pgm_clients[service_name]
            if client.wait_for_service(timeout_sec=self.pgm_wait_timeout):
                return service_name, client
        return None, None
    
    def get_map_name(self, custom_name: str) -> str:
        """获取地图名称，如果为空则使用时间戳"""
        if custom_name and custom_name.strip():
            return custom_name.strip()
        return datetime.now().strftime('%Y%m%d_%H%M%S')
    
    def save_all_maps_callback(self, request, response):
        """保存所有地图的回调函数"""
        map_name = self.get_map_name(request.map_name)
        response.success = True
        response.message = ''
        response.pgm_path = ''
        response.pcd_path = ''
        
        self.get_logger().info(f'Saving maps with name: {map_name}')
        
        # 保存 PGM
        if request.save_pgm:
            pgm_result = self.save_pgm(map_name)
            if pgm_result['success']:
                response.pgm_path = pgm_result['path']
                response.message += f"PGM saved to {pgm_result['path']}. "
            else:
                response.success = False
                response.message += f"PGM save failed: {pgm_result['error']}. "
        
        # 保存 PCD
        if request.save_pcd:
            pcd_result = self.save_pcd(map_name, request.save_patches)
            if pcd_result['success']:
                response.pcd_path = pcd_result['path']
                response.message += f"PCD saved to {pcd_result['path']}. "
            else:
                response.success = False
                response.message += f"PCD save failed: {pcd_result['error']}. "
        
        if response.success:
            self.get_logger().info(f'All maps saved successfully: {response.message}')
        else:
            self.get_logger().error(f'Map save failed: {response.message}')
        
        return response
    
    def save_pgm(self, map_name: str) -> dict:
        """保存 PGM 地图"""
        # 创建目录
        pgm_dir = os.path.join(self.pgm_base, map_name)
        try:
            os.makedirs(pgm_dir, exist_ok=True)
        except Exception as e:
            return {'success': False, 'error': str(e), 'path': ''}
        
        # 检查服务是否可用
        service_name, pgm_client = self._find_available_pgm_client()
        if pgm_client is None:
            return {
                'success': False,
                'error': f'PGM save service not available, tried: {self.pgm_service_names}',
                'path': ''
            }
        
        # 构建请求
        request = SaveMap.Request()
        request.map_topic = 'map'
        request.map_url = os.path.join(pgm_dir, 'map')
        request.image_format = 'pgm'
        request.map_mode = 'trinary'
        request.free_thresh = 0.25
        request.occupied_thresh = 0.65

        # 调用服务
        try:
            self.get_logger().info(f'Calling PGM save service: {service_name}')
            future = pgm_client.call_async(request)
            rclpy.spin_until_future_complete(
                self,
                future,
                executor=self.executor,
                timeout_sec=self.pgm_call_timeout
            )

            if not future.done():
                return {
                    'success': False,
                    'error': f'PGM save service timeout ({self.pgm_call_timeout}s)',
                    'path': ''
                }

            result = future.result()
            if result and result.result:
                return {'success': True, 'error': '', 'path': pgm_dir}
            else:
                return {'success': False, 'error': 'PGM save returned false', 'path': ''}
        except Exception as e:
            return {'success': False, 'error': str(e), 'path': ''}
    
    def save_pcd(self, map_name: str, save_patches: bool) -> dict:
        """保存 PCD 地图"""
        # 创建目录
        pcd_dir = os.path.join(self.pcd_base, map_name)
        try:
            os.makedirs(pcd_dir, exist_ok=True)
        except Exception as e:
            return {'success': False, 'error': str(e), 'path': ''}
        
        # 检查服务是否可用
        if not self.pcd_client.wait_for_service(timeout_sec=self.pcd_wait_timeout):
            return {'success': False, 'error': 'PCD save service not available', 'path': ''}
        
        # 构建请求
        request = SaveMaps.Request()
        request.file_path = pcd_dir
        request.save_patches = save_patches
        
        # 调用服务
        try:
            future = self.pcd_client.call_async(request)
            rclpy.spin_until_future_complete(
                self,
                future,
                executor=self.executor,
                timeout_sec=self.pcd_call_timeout
            )

            if not future.done():
                return {
                    'success': False,
                    'error': f'PCD save service timeout ({self.pcd_call_timeout}s)',
                    'path': ''
                }

            result = future.result()
            if result and result.success:
                return {'success': True, 'error': '', 'path': pcd_dir}
            else:
                error_msg = result.message if result else 'PCD save service returned empty response'
                return {'success': False, 'error': error_msg, 'path': ''}
        except Exception as e:
            return {'success': False, 'error': str(e), 'path': ''}


def main(args=None):
    rclpy.init(args=args)
    node = MapSaveServiceNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
