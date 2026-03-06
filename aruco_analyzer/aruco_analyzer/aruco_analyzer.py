import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage, CameraInfo
from aruco_msgs.msg import ArucoMarker, ArucoMarkers   # кастомные сообщения
from visualization_msgs.msg import Marker, MarkerArray
import cv2
import cv2.aruco as aruco
import numpy as np
from cv_bridge import CvBridge
import math
from geometry_msgs.msg import Quaternion, Pose

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')

        # Параметры
        self.declare_parameter('enable_debug_flow', False)
        self.declare_parameter('enable_rviz_markers', False)
        self.declare_parameter('aruco_dictionary', 'DICT_6X6_250')
        self.declare_parameter('marker_size', 0.1)  # метры
        self.declare_parameter('camera_topic', '/camera/image_raw/compressed')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('output_image_topic', '/aruco/debug_image/compressed')
        self.declare_parameter('output_poses_topic', '/aruco/markers')
        self.declare_parameter('rviz_markers_topic', '/aruco/markers_rviz')

        self.enable_debug = self.get_parameter('enable_debug_flow').value
        self.enable_rviz = self.get_parameter('enable_rviz_markers').value
        self.marker_size = self.get_parameter('marker_size').value
        self.dict_name = self.get_parameter('aruco_dictionary').value

        # Словарь Aruco
        self.aruco_dict = self._get_aruco_dict(self.dict_name)
        self.aruco_params = aruco.DetectorParameters()

        # Калибровочные данные камеры (будут получены из топика)
        self.camera_matrix = None
        self.dist_coeffs = None

        # Bridge для конвертации CompressedImage <-> OpenCV
        self.bridge = CvBridge()

        # Подписки
        qos_sensor = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub_img = self.create_subscription(
            CompressedImage,
            self.get_parameter('camera_topic').value,
            self.image_callback,
            qos_sensor
        )
        self.sub_cam_info = self.create_subscription(
            CameraInfo,
            self.get_parameter('camera_info_topic').value,
            self.camera_info_callback,
            qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )

        # Издатели
        if self.enable_debug:
            self.pub_debug_img = self.create_publisher(
                CompressedImage,
                self.get_parameter('output_image_topic').value,
                10
            )

        # Издатель для кастомного сообщения ArucoMarkers
        self.pub_aruco_markers = self.create_publisher(
            ArucoMarkers,
            self.get_parameter('output_poses_topic').value,
            10
        )

        if self.enable_rviz:
            self.marker_array_pub = self.create_publisher(
                MarkerArray,
                self.get_parameter('rviz_markers_topic').value,
                10
            )

        self.get_logger().info('Aruco detector node started.')

    def _get_aruco_dict(self, name):
        """Возвращает словарь OpenCV Aruco по имени."""
        dictionaries = {
            'DICT_4X4_50': aruco.DICT_4X4_50,
            'DICT_4X4_100': aruco.DICT_4X4_100,
            'DICT_4X4_250': aruco.DICT_4X4_250,
            'DICT_4X4_1000': aruco.DICT_4X4_1000,
            'DICT_5X5_50': aruco.DICT_5X5_50,
            'DICT_5X5_100': aruco.DICT_5X5_100,
            'DICT_5X5_250': aruco.DICT_5X5_250,
            'DICT_5X5_1000': aruco.DICT_5X5_1000,
            'DICT_6X6_50': aruco.DICT_6X6_50,
            'DICT_6X6_100': aruco.DICT_6X6_100,
            'DICT_6X6_250': aruco.DICT_6X6_250,
            'DICT_6X6_1000': aruco.DICT_6X6_1000,
            'DICT_7X7_50': aruco.DICT_7X7_50,
            'DICT_7X7_100': aruco.DICT_7X7_100,
            'DICT_7X7_250': aruco.DICT_7X7_250,
            'DICT_7X7_1000': aruco.DICT_7X7_1000,
            'DICT_ARUCO_ORIGINAL': aruco.DICT_ARUCO_ORIGINAL,
        }
        if name not in dictionaries:
            self.get_logger().warn(f'Unknown dictionary {name}, using DICT_6X6_250')
            name = 'DICT_6X6_250'
        return aruco.getPredefinedDictionary(dictionaries[name])

    def camera_info_callback(self, msg: CameraInfo):
        """Сохраняем матрицу камеры и коэффициенты искажения."""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)
        self.get_logger().info('Camera calibration received.')

    def image_callback(self, msg: CompressedImage):
        """Основная обработка кадра."""
        try:
            # Декодируем сжатое изображение в OpenCV (BGR)
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().error(f'Failed to decode image: {e}')
            return

        # Обнаружение маркеров
        corners, ids, rejected = aruco.detectMarkers(
            cv_image, self.aruco_dict, parameters=self.aruco_params
        )

        debug_image = cv_image.copy()
        aruco_markers_msg = ArucoMarkers()  # контейнер для кастомных сообщений

        if ids is not None and len(ids) > 0 and self.camera_matrix is not None:
            # Оценка позы для каждого маркера
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coeffs
            )

            # Подготовка массива маркеров для RVIZ (если включено)
            if self.enable_rviz:
                marker_array = MarkerArray()

            for i, marker_id in enumerate(ids.flatten()):
                # Позиция и поворот
                tvec = tvecs[i][0]
                rvec = rvecs[i][0]
                rot_matrix, _ = cv2.Rodrigues(rvec)
                q = self.rotation_matrix_to_quaternion(rot_matrix)

                # Заполняем кастомное сообщение ArucoMarker
                aruco_marker = ArucoMarker()
                aruco_marker.header = msg.header  # копируем заголовок исходного изображения
                aruco_marker.id = int(marker_id)
                aruco_marker.pose.position.x = float(tvec[0])
                aruco_marker.pose.position.y = float(tvec[1])
                aruco_marker.pose.position.z = float(tvec[2])
                aruco_marker.pose.orientation = q
                aruco_marker.marker_length = self.marker_size
                aruco_marker.dictionary = self.dict_name

                aruco_markers_msg.markers.append(aruco_marker)

                # Если debug включён, рисуем на изображении
                if self.enable_debug:
                    aruco.drawDetectedMarkers(debug_image, corners, ids)
                    cv2.drawFrameAxes(debug_image, self.camera_matrix, self.dist_coeffs,
                                      rvec, tvec, self.marker_size * 0.5)

                # Если включена публикация в RVIZ, создаём визуальные маркеры
                if self.enable_rviz:
                    # Маркер-куб для отображения метки
                    cube_marker = self._create_cube_marker(
                        marker_id, msg.header.frame_id, tvec, q, msg.header.stamp
                    )
                    marker_array.markers.append(cube_marker)

                    # Текстовый маркер с ID (над кубом)
                    text_marker = self._create_text_marker(
                        marker_id, msg.header.frame_id, tvec, q, msg.header.stamp
                    )
                    marker_array.markers.append(text_marker)

            if self.enable_rviz and ids is not None and len(ids) > 0:
                # Устанавливаем время жизни для всех маркеров (0.1 секунды)
                for m in marker_array.markers:
                    m.lifetime.sec = 0
                    m.lifetime.nanosec = 100000000  # 0.1 сек
                self.marker_array_pub.publish(marker_array)

        # Публикуем кастомное сообщение с маркерами (даже пустое, если ничего не найдено)
        self.pub_aruco_markers.publish(aruco_markers_msg)

        # Публикация отладочного изображения, если включено
        if self.enable_debug:
            _, encoded_img = cv2.imencode('.jpg', debug_image, [cv2.IMWRITE_JPEG_QUALITY, 85])
            debug_msg = CompressedImage()
            debug_msg.header = msg.header
            debug_msg.format = 'jpeg'
            debug_msg.data = encoded_img.tobytes()
            self.pub_debug_img.publish(debug_msg)

    def _create_cube_marker(self, marker_id, frame_id, tvec, q, stamp):
        """Создаёт маркер типа CUBE для отображения метки в RVIZ."""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = 'aruco_cubes'
        marker.id = int(marker_id)
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = float(tvec[0])
        marker.pose.position.y = float(tvec[1])
        marker.pose.position.z = float(tvec[2])
        marker.pose.orientation = q
        # Размер куба соответствует размеру метки, но делаем небольшую толщину
        marker.scale.x = self.marker_size
        marker.scale.y = self.marker_size
        marker.scale.z = 0.01  # толщина
        marker.color.a = 0.8
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        return marker

    def _create_text_marker(self, marker_id, frame_id, tvec, q, stamp):
        """Создаёт маркер с текстом (ID метки), расположенный над кубом."""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = 'aruco_text'
        marker.id = int(marker_id)
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        # Позиция текста - над центром куба (смещение по z на половину высоты куба + небольшой отступ)
        marker.pose.position.x = float(tvec[0])
        marker.pose.position.y = float(tvec[1])
        marker.pose.position.z = float(tvec[2]) + 0.05  # поднят на 5 см
        # Ориентация не важна для TEXT_VIEW_FACING
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1  # Размер текста
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.text = f"ID: {marker_id}"
        return marker

    @staticmethod
    def rotation_matrix_to_euler(R):
        """Преобразование матрицы поворота в углы Эйлера (ZYX): roll, pitch, yaw."""
        sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
        singular = sy < 1e-6

        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0

        return x, y, z

    @staticmethod
    def rotation_matrix_to_quaternion(R):
        """Преобразование матрицы поворота в кватернион (geometry_msgs/Quaternion)."""
        q = Quaternion()
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        if trace > 0:
            s = 0.5 / math.sqrt(trace + 1.0)
            q.w = 0.25 / s
            q.x = (R[2, 1] - R[1, 2]) * s
            q.y = (R[0, 2] - R[2, 0]) * s
            q.z = (R[1, 0] - R[0, 1]) * s
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
                q.w = (R[2, 1] - R[1, 2]) / s
                q.x = 0.25 * s
                q.y = (R[0, 1] + R[1, 0]) / s
                q.z = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
                q.w = (R[0, 2] - R[2, 0]) / s
                q.x = (R[0, 1] + R[1, 0]) / s
                q.y = 0.25 * s
                q.z = (R[1, 2] + R[2, 1]) / s
            else:
                s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
                q.w = (R[1, 0] - R[0, 1]) / s
                q.x = (R[0, 2] + R[2, 0]) / s
                q.y = (R[1, 2] + R[2, 1]) / s
                q.z = 0.25 * s
        return q


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()