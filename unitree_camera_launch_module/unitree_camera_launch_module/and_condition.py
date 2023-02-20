from typing import Iterable
from typing import Text

from launch.condition import Condition
from launch.launch_context import LaunchContext

class AndCondition(Condition):
    """Condition that returns true if all of the input condition list are true"""

    def __init__(self, conditions: Iterable[Condition]) -> None:
        """Create an OrCondition"""
        self.__conditions = conditions
        super().__init__(predicate=self._predicate_func)

    def _predicate_func(self, context: LaunchContext) -> bool:
        # if any of the input conditions are false, return false
        for condition in self.__conditions:
            if not condition.evaluate(context):
                return False
            
        # otherwise return True
        return True

    def describe(self) -> Text:
        """Return a description of this Condition."""
        return self.__repr__()