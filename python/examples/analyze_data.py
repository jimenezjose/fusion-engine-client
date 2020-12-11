from argparse import ArgumentParser
import logging
import os
import sys

root_dir = os.path.normpath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(root_dir)

from fusion_engine_client.analysis.file_reader import FileReader
from fusion_engine_client.messages.core import *


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument('file', type=str, help="The path to a binary file to be read.")
    options = parser.parse_args()

    logging.basicConfig(format='%(levelname)s - %(name)s:%(lineno)d - %(message)s')
    logger = logging.getLogger('point_one.fusion_engine')
    logger.setLevel(logging.DEBUG)

    # Read pose data from the file.
    reader = FileReader(options.file)
    result = reader.read(message_types=[PoseMessage])

    # Print out the messages that were read.
    pose_data = result[PoseMessage.MESSAGE_TYPE]
    for message in pose_data.messages:
        logger.info(str(message))
