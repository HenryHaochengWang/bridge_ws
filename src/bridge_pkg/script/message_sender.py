#!/usr/bin/env python3

import rospy
import websockets
import json
import asyncio
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker

URI = 'ws://131.170.250.219:9090'


async def send_data(uri, data):
    async with websockets.connect(uri) as ws:
        await ws.send(json.dumps(data))

async def advertise(uri, topic, type):
    message = {
        'op': 'advertise',
        'topic': topic,
        'type': type
    }
    await send_data(uri, message)

def send_robot_direction():
    rospy.loginfo("Fetching robot_description")
    robot_description = rospy.get_param('/robot_description')
    message = {
        'op': 'publish',
        'topic': '/robot_description',
        'msg': robot_description
    }
    asyncio.run(send_data(URI, message))

def marker_msgs_callback(data):
    rospy.loginfo("Received Marker message")
    message = {
        'op': 'publish',
        'topic': '/visualization_marker',
        'msg': {
            'header': {
                'frame_id': data.header.frame_id,
                'stamp': {
                    'secs': data.header.stamp.secs,
                    'nsecs': data.header.stamp.nsecs
                }
            },
            'ns': data.ns,
            'id': data.id,
            'type': data.type,
            'action': data.action,
            'pose': {
                'position': {
                    'x': data.pose.position.x,
                    'y': data.pose.position.y,
                    'z': data.pose.position.z,
                },
                'orientation': {
                    'x': data.pose.orientation.x,
                    'y': data.pose.orientation.y,
                    'z': data.pose.orientation.z,
                    'w': data.pose.orientation.w,
                },
            },
            'scale': {
                'x': data.scale.x,
                'y': data.scale.y,
                'z': data.scale.z,
            },
            'color': {
                'r': data.color.r,
                'g': data.color.g,
                'b': data.color.b,
                'a': data.color.a,
            },
            'lifetime': {
                'secs': data.lifetime.secs,
                'nsecs': data.lifetime.nsecs
            },
            'frame_locked': data.frame_locked,
            'points': [{'x': p.x, 'y': p.y, 'z': p.z} for p in data.points],
            'colors': [{'r': c.r, 'g': c.g, 'b': c.b, 'a': c.a} for c in data.colors],
            'text': data.text,
            'mesh_resource': data.mesh_resource,
            'mesh_use_embedded_materials': data.mesh_use_embedded_materials
        }
    }
    asyncio.run(send_data(URI, message))

def image_msgs_callback(data):
    rospy.loginfo("Received Image message")
    message = {
        'op': 'publish',
        'topic': '/xtion/rgb/image_raw',
        'msg': {
            'header': {
                'frame_id': data.header.frame_id,
                'stamp': {
                    'secs': data.header.stamp.secs,
                    'nsecs': data.header.stamp.nsecs
                }
            },
            'height': data.height,
            'width': data.width,
            'encoding': data.encoding,
            'is_bigendian': data.is_bigendian,
            'step': data.step,
            'data': list(data.data)  # Ensure the data is a list for JSON serialization
        }
    }
    asyncio.run(send_data(URI, message))

def compressed_image_msgs_callback(data):
    rospy.loginfo("Received Compressed Image message")
    message = {
        'op': 'publish',
        'topic': '/xtion/rgb/image_raw/compressed',
        'msg': {
            'header': {
                'frame_id': data.header.frame_id,
                'stamp': {
                    'secs': data.header.stamp.secs,
                    'nsecs': data.header.stamp.nsecs
                }
            },
            'format': data.format,
            'data': list(data.data)  # Ensure the data is a list for JSON serialization
        }
    }
    asyncio.run(send_data(URI, message))

def map_msgs_callback(data):
    rospy.loginfo("Received Map message")
    message = {
        'op': 'publish',
        'topic': '/map',
        'msg': {
            'header': {
                'frame_id': data.header.frame_id,
                'stamp': {
                    'secs': data.header.stamp.secs,
                    'nsecs': data.header.stamp.nsecs
                }
            },
            'info': {
                'map_load_time': {
                    'secs': data.info.map_load_time.secs,
                    'nsecs': data.info.map_load_time.nsecs
                },
                'resolution': data.info.resolution,
                'width': data.info.width,
                'height': data.info.height,
                'origin': {
                    'position': {
                        'x': data.info.origin.position.x,
                        'y': data.info.origin.position.y,
                        'z': data.info.origin.position.z,
                    },
                    'orientation': {
                        'x': data.info.origin.orientation.x,
                        'y': data.info.origin.orientation.y,
                        'z': data.info.origin.orientation.z,
                        'w': data.info.origin.orientation.w,
                    },
                },
            },
            'data': list(data.data)  # Ensure the data is a list for JSON serialization
        }
    }
    asyncio.run(send_data(URI, message))

def listener():
    rospy.init_node('message_sender', anonymous=True)

    asyncio.run(advertise(URI, '/robot_description', 'std_msgs/String'))
    asyncio.run(advertise(URI, '/visualization_marker', 'visualization_msgs/Marker'))
    # asyncio.run(advertise(URI, '/xtion/rgb/image_raw', 'sensor_msgs/Image'))
    asyncio.run(advertise(URI, '/xtion/rgb/image_raw/compressed', 'sensor_msgs/CompressedImage'))
    asyncio.run(advertise(URI, '/map', 'nav_msgs/OccupancyGrid'))

    send_robot_direction()

    rospy.Subscriber('/visualization_marker', Marker, marker_msgs_callback)
    # rospy.Subscriber('/xtion/rgb/image_raw', Image, image_msgs_callback)
    rospy.Subscriber('/xtion/rgb/image_raw/compressed', CompressedImage, compressed_image_msgs_callback)
    rospy.Subscriber('/map', OccupancyGrid, map_msgs_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()