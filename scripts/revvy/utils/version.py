# SPDX-License-Identifier: GPL-3.0-only
import re


version_re = re.compile('(?P<major>\\d+?)\\.(?P<minor>\\d+?)(\\.(?P<rev>\\d+))?(-.*?)?$')


class FormatError(Exception):
    pass


class Version:
    def __init__(self, ver_str):
        """
        >>> Version('1.0.123')
        Version(1.0.123)
        >>> Version('1.0-foobar') # optional tag is ignored
        Version(1.0.0)
        """
        match = version_re.match(ver_str)
        if not match:
            raise FormatError
        self._major = int(match.group('major'))
        self._minor = int(match.group('minor'))
        self._rev = int(match.group('rev')) if match.group('rev') else 0
        self._normalized = '{}.{}.{}'.format(self._major, self._minor, self._rev)

    @property
    def major(self):
        """
        >>> Version('2.3').major
        2
        """
        return self._major

    @property
    def minor(self):
        """
        >>> Version('2.3').minor
        3
        """
        return self._minor

    @property
    def revision(self):
        """
        >>> Version('2.3.45').revision
        45
        """
        return self._rev

    def __le__(self, other):
        """
        >>> Version('1.0.0') <= Version('1.0.0')
        True
        >>> Version('1.0.0') <= Version('1.0.1')
        True
        >>> Version('1.0.0') <= Version('1.1.0')
        True
        >>> Version('1.0.0') <= Version('2.0.0')
        True
        >>> Version('1.0.1') <= Version('1.0.0')
        False
        >>> Version('1.1.0') <= Version('1.0.0')
        False
        >>> Version('2.0.0') <= Version('1.0.0')
        False
        """
        return self.compare(other) != 1

    def __eq__(self, other):
        """
        >>> Version('1.0.0') == Version('1.0.0')
        True
        >>> Version('1.0.0') == Version('1.0.1')
        False
        >>> Version('1.0.0') == Version('1.1.0')
        False
        >>> Version('1.0.0') == Version('2.0.0')
        False
        """
        return self.compare(other) == 0

    def __ne__(self, other):
        """
        >>> Version('1.0.0') != Version('1.0.0')
        False
        >>> Version('1.0.0') != Version('1.0.1')
        True
        >>> Version('1.0.0') != Version('1.1.0')
        True
        >>> Version('1.0.0') != Version('2.0.0')
        True
        """
        return self.compare(other) != 0

    def __lt__(self, other):
        """
        >>> Version('1.0.0') < Version('1.0.0')
        False
        >>> Version('1.0.0') < Version('1.0.1')
        True
        >>> Version('1.0.0') < Version('1.1.0')
        True
        >>> Version('1.0.0') < Version('2.0.0')
        True
        >>> Version('1.0.1') < Version('1.0.0')
        False
        >>> Version('1.1.0') < Version('1.0.0')
        False
        >>> Version('2.0.0') < Version('1.0.0')
        False
        """
        return self.compare(other) == -1

    def __gt__(self, other):
        """
        >>> Version('1.0.0') > Version('1.0.0')
        False
        >>> Version('1.0.0') > Version('1.0.1')
        False
        >>> Version('1.0.0') > Version('1.1.0')
        False
        >>> Version('1.0.0') > Version('2.0.0')
        False
        >>> Version('1.0.1') > Version('1.0.0')
        True
        >>> Version('1.1.0') > Version('1.0.0')
        True
        >>> Version('2.0.0') > Version('1.0.0')
        True
        """
        return self.compare(other) == 1

    def __ge__(self, other):
        """
        >>> Version('1.0.0') >= Version('1.0.0')
        True
        >>> Version('1.0.0') >= Version('1.0.1')
        False
        >>> Version('1.0.0') >= Version('1.1.0')
        False
        >>> Version('1.0.0') >= Version('2.0.0')
        False
        >>> Version('1.0.1') >= Version('1.0.0')
        True
        >>> Version('1.1.0') >= Version('1.0.0')
        True
        >>> Version('2.0.0') >= Version('1.0.0')
        True
        """
        return self.compare(other) != -1

    # noinspection PyProtectedMember
    def compare(self, other):
        """
        >>> Version('1.0.0').compare(Version('1.0.0'))
        0
        >>> Version('1.0.1').compare(Version('1.0.0'))
        1
        >>> Version('1.0.0').compare(Version('1.0.1'))
        -1
        """

        def cmp(a, b):
            return -1 if a < b else 1

        if self._major == other._major:
            if self._minor == other._minor:
                if self._rev == other._rev:
                    return 0
                else:
                    return cmp(self._rev, other._rev)
            else:
                return cmp(self._minor, other._minor)
        else:
            return cmp(self._major, other._major)

    def __str__(self) -> str:
        """
        >>> str(Version('1.0'))
        '1.0.0'
        """
        return self._normalized

    def __repr__(self):
        return 'Version({})'.format(self._normalized)

    def __hash__(self) -> int:
        return self._normalized.__hash__()
