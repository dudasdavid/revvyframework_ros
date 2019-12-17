# SPDX-License-Identifier: GPL-3.0-only

import hashlib
import json
import traceback
from binascii import b2a_base64, a2b_base64

import math


def clip(x, min_x, max_x):
    """Constrain a number between two limits

    >>> clip(3, 1, 2)
    2
    >>> clip(0, 1, 2)
    1
    >>> clip(1.5, 1, 2)
    1.5
    """
    return min(max(x, min_x), max_x)


def map_values(x, min_x, max_x, min_y, max_y):
    """Scales a number from the input range of [min_x, max_x] to between [min_y, max_y]

    >>> map_values(0.5, 0, 1, 0, 900)
    450.0
    >>> map_values(math.pi/2, 0, math.pi, 0, 180)
    90.0
    >>> map_values(8, 0, 10, 5, 0)
    1.0
    """
    full_scale_in = max_x - min_x
    full_scale_out = max_y - min_y
    return (x - min_x) * (full_scale_out / full_scale_in) + min_y


def get_serial():
    """Extract serial from cpuinfo file"""

    cpu_serial = "0000000000000000"
    # noinspection PyBroadException
    try:
        with open('/proc/cpuinfo', 'r') as f:
            for line in f:
                if line.startswith('Serial'):
                    cpu_serial = line.rstrip()[-16:].lstrip('0')
                    break
    except Exception:
        print('Failed to read cpuid')
        print(traceback.format_exc())
        cpu_serial = "ERROR000000000"

    return cpu_serial


def retry(fn, retries=5):
    """Retry the given function a number of times, or until it returns True or None"""
    status = False
    retry_num = 0
    while retry_num < retries and not status:
        # noinspection PyBroadException
        try:
            status = fn()
            if status is None:
                status = True
        except Exception:
            print(traceback.format_exc())
            status = False
        retry_num += 1

    return status


def split(data, chunk_size):
    """
    >>> list(split([], 5))
    []
    >>> list(split(b'apple', 5))
    [b'apple']
    >>> list(split(b'apple', 7))
    [b'apple']
    >>> list(split([1, 2, 3, 4], 2))
    [[1, 2], [3, 4]]
    >>> list(split([1, 2, 3, 4, 5], 2))
    [[1, 2], [3, 4], [5]]
    >>> list(split(b'apple', 3))
    [b'app', b'le']
    """
    return (data[i:i + chunk_size] for i in range(0, len(data), chunk_size))


def hex2rgb(hex_str):
    """
    >>> hex2rgb("#aabbcc")
    11189196
    >>> hex2rgb("#000000")
    0
    >>> hex2rgb("#0000FF")
    255
    """
    rgb = [int(chunk, 16) for chunk in split(hex_str.lstrip('#'), 2)]

    return rgb[0] << 16 | rgb[1] << 8 | rgb[2]


def b64_encode_str(to_encode):
    """
    >>> b64_encode_str("hello")
    'aGVsbG8='
    """
    return b2a_base64(to_encode.encode("utf-8")).decode("utf-8").rstrip()


def b64_decode_str(to_decode):
    """
    >>> b64_decode_str('aGVsbG8=')
    'hello'
    """
    return a2b_base64(to_decode.encode("utf-8")).decode("utf-8")


def is_bit_set(reg, bit):
    """
    >>> is_bit_set(1, 0)
    True
    >>> is_bit_set(2, 1)
    True
    >>> is_bit_set(3, 1)
    True
    """
    return reg & (1 << bit) != 0


def bits_to_bool_list(byte_list):
    """
    >>> bits_to_bool_list([0xFF])
    [True, True, True, True, True, True, True, True]
    >>> bits_to_bool_list([0x01, 0x00])
    [True, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False]
    >>> bits_to_bool_list([0x00, 0x80])
    [False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, True]
    """

    def expand_byte(byte):
        return [is_bit_set(byte, x) for x in range(8)]

    return [j for i in [expand_byte(byte) for byte in byte_list] for j in i]


def dict_get_first(dictionary: dict, keys: list):
    """
    Read a value from a dictionary, using multiple possible keys

    >>> dict_get_first({'a': 'foo', 'b': 'bar', 'c': 'foobar'}, ['b', 'c'])
    'bar'
    >>> dict_get_first({'a': 'foo', 'b': 'bar', 'c': 'foobar'}, ['d', 'c'])
    'foobar'
    >>> dict_get_first({'a': 'foo', 'b': 'bar', 'c': 'foobar'}, ['e', 'f'])
    Traceback (most recent call last):
    ...
    KeyError: 'e, f'
    """
    for key in keys:
        try:
            return dictionary[key]
        except KeyError:
            pass
    raise KeyError(', '.join(keys))


# noinspection SpellCheckingInspection
def bytestr_hash(byte_str):
    hash_fn = hashlib.md5()
    hash_fn.update(byte_str)
    return hash_fn.hexdigest()


def file_hash(file):
    with open(file, "rb") as f:
        return bytestr_hash(f.read())


def read_json(filename):
    with open(filename, "r") as f:
        return json.load(f)
