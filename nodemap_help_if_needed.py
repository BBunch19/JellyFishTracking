# NODE HELP------------------------
MAX_CHARS = 35


class ReadType:
    """
    Use the following constants to determine whether nodes are read
    as Value nodes or their individual types.
    """
    VALUE = 0,
    INDIVIDUAL = 1

CHOSEN_READ = ReadType.INDIVIDUAL
def print_with_indent(level, text):
    """
    Helper function for printing a string prefix with a specifc number of indents.
    :param level: Number of indents to generate
    :type level: int
    :param text: String to print after indent
    :type text: str
    """
    ind = ''
    for i in range(level):
        ind += '    '
    print('%s%s' % (ind, text))


def print_value_node(node, level):
    """
    Retrieves and prints the display name and value of all node types as value nodes.
    A value node is a general node type that allows for the reading and writing of any node type as a string.

    :param node: Node to get information from.
    :type node: INode
    :param level: Depth to indent output.
    :type level: int
    :return: True if successful, False otherwise.
    :rtype: bool
    """
    try:
        result = True

        # Create value node
        node_value = PySpin.CValuePtr(node)

        # Retrieve display name
        #
        # *** NOTES ***
        # A node's 'display name' is generally more appropriate for output and
        # user interaction whereas its 'name' is what the camera understands.
        # Generally, its name is the same as its display name but without
        # spaces - for instance, the name of the node that houses a camera's
        # serial number is 'DeviceSerialNumber' while its display name is
        # 'Device Serial Number'.
        display_name = node_value.GetDisplayName()

        # Retrieve value of any node type as string
        #
        # *** NOTES ***
        # Because value nodes return any node type as a string, it can be much
        # easier to deal with nodes as value nodes rather than their actual
        # individual types.
        value = node_value.ToString()

        # Cap length at MAX_CHARS
        value = value[:MAX_CHARS] + '...' if len(value) > MAX_CHARS else value

        # Print value
        print_with_indent(level, '%s: %s' % (display_name, value))

    except PySpin.SpinnakerException as ex:
        print('Error: %s' % ex)
        return False

    return result


def print_string_node(node, level):
    """
    Retrieves and prints the display name and value of a string node.

    :param node: Node to get information from.
    :type node: INode
    :param level: Depth to indent output.
    :type level: int
    :return: True if successful, False otherwise.
    :rtype: bool
    """
    try:
        result = True

        # Create string node
        node_string = PySpin.CStringPtr(node)

        # Retrieve string node value
        #
        # *** NOTES ***
        # Functions in Spinnaker C++ that use gcstring types
        # are substituted with Python strings in PySpin.
        # The only exception is shown in the DeviceEvents example, where
        # the callback function still uses a wrapped gcstring type.
        display_name = node_string.GetDisplayName()

        # Ensure that the value length is not excessive for printing
        value = node_string.GetValue()
        value = value[:MAX_CHARS] + '...' if len(value) > MAX_CHARS else value

        # Print value; 'level' determines the indentation level of output
        print_with_indent(level, '%s: %s' % (display_name, value))

    except PySpin.SpinnakerException as ex:
        print('Error: %s' % ex)
        return False

    return result


def print_integer_node(node, level):
    """
    Retrieves and prints the display name and value of an integer node.

    :param node: Node to get information from.
    :type node: INode
    :param level: Depth to indent output.
    :type level: int
    :return: True if successful, False otherwise.
    :rtype: bool
    """
    try:
        result = True

        # Create integer node
        node_integer = PySpin.CIntegerPtr(node)

        # Get display name
        display_name = node_integer.GetDisplayName()

        # Retrieve integer node value
        #
        # *** NOTES ***
        # All node types except base nodes have a ToString()
        # method which returns a value as a string.
        value = node_integer.GetValue()

        # Print value
        print_with_indent(level, '%s: %s' % (display_name, value))

    except PySpin.SpinnakerException as ex:
        print('Error: %s' % ex)
        return False

    return result


def print_float_node(node, level):
    """
    Retrieves and prints the display name and value of a float node.

    :param node: Node to get information from.
    :type node: INode
    :param level: Depth to indent output.
    :type level: int
    :return: True if successful, False otherwise.
    :rtype: bool
    """
    try:
        result = True

        # Create float node
        node_float = PySpin.CFloatPtr(node)

        # Get display name
        display_name = node_float.GetDisplayName()

        # Retrieve float value
        value = node_float.GetValue()

        # Print value
        print_with_indent(level, '%s: %s' % (display_name, value))

    except PySpin.SpinnakerException as ex:
        print('Error: %s' % ex)
        return False

    return result


def print_boolean_node(node, level):
    """
    Retrieves and prints the display name and value of a Boolean node.

    :param node: Node to get information from.
    :type node: INode
    :param level: Depth to indent output.
    :type level: int
    :return: True if successful, False otherwise.
    :rtype: bool
    """
    try:
        result = True

        # Create Boolean node
        node_boolean = PySpin.CBooleanPtr(node)

        # Get display name
        display_name = node_boolean.GetDisplayName()

        # Retrieve Boolean value
        value = node_boolean.GetValue()

        # Print Boolean value
        # NOTE: In Python a Boolean will be printed as "True" or "False".
        print_with_indent(level, '%s: %s' % (display_name, value))

    except PySpin.SpinnakerException as ex:
        print('Error: %s' % ex)
        return False

    return result


def print_command_node(node, level):
    """
    This function retrieves and prints the display name and tooltip of a command
    node, limiting the number of printed characters to a macro-defined maximum.
    The tooltip is printed below because command nodes do not have an intelligible
    value.

    :param node: Node to get information from.
    :type node: INode
    :param level: Depth to indent output.
    :type level: int
    :return: True if successful, False otherwise.
    :rtype: bool
    """
    try:
        result = True

        # Create command node
        node_command = PySpin.CCommandPtr(node)

        # Get display name
        display_name = node_command.GetDisplayName()

        # Retrieve tooltip
        #
        # *** NOTES ***
        # All node types have a tooltip available. Tooltips provide useful
        # information about nodes. Command nodes do not have a method to
        # retrieve values as their is no intelligible value to retrieve.
        tooltip = node_command.GetToolTip()

        # Ensure that the value length is not excessive for printing
        tooltip = tooltip[:MAX_CHARS] + '...' if len(tooltip) > MAX_CHARS else tooltip

        # Print display name and tooltip
        print_with_indent(level, '%s: %s' % (display_name, tooltip))

    except PySpin.SpinnakerException as ex:
        print('Error: %s' % ex)
        return False

    return result


def print_enumeration_node_and_current_entry(node, level):
    """
    This function retrieves and prints the display names of an enumeration node
    and its current entry (which is actually housed in another node unto itself).

    :param node: Node to get information from.
    :type node: INode
    :param level: Depth to indent output.
    :type level: int
    :return: True if successful, False otherwise.
    :rtype: bool
    """
    try:
        result = True

        # Create enumeration node
        node_enumeration = PySpin.CEnumerationPtr(node)

        # Retrieve current entry as enumeration node
        #
        # *** NOTES ***
        # Enumeration nodes have three methods to differentiate between: first,
        # GetIntValue() returns the integer value of the current entry node;
        # second, GetCurrentEntry() returns the entry node itself; and third,
        # ToString() returns the symbolic of the current entry.
        node_enum_entry = PySpin.CEnumEntryPtr(node_enumeration.GetCurrentEntry())

        # Get display name
        display_name = node_enumeration.GetDisplayName()

        # Retrieve current symbolic
        #
        # *** NOTES ***
        # Rather than retrieving the current entry node and then retrieving its
        # symbolic, this could have been taken care of in one step by using the
        # enumeration node's ToString() method.
        entry_symbolic = node_enum_entry.GetSymbolic()

        # Print current entry symbolic
        print_with_indent(level, '%s: %s' % (display_name, entry_symbolic))

    except PySpin.SpinnakerException as ex:
        print('Error: %s' % ex)
        return False

    return result


def print_category_node_and_all_features(node, level):
    """
    This function retrieves and prints out the display name of a category node
    before printing all child nodes. Child nodes that are also category nodes are
    printed recursively.

    :param node: Category node to get information from.
    :type node: INode
    :param level: Depth to indent output.
    :type level: int
    :return: True if successful, False otherwise.
    :rtype: bool
    """
    try:
        result = True

        # Create category node
        node_category = PySpin.CCategoryPtr(node)

        # Get and print display name
        display_name = node_category.GetDisplayName()
        print_with_indent(level, display_name)

        # Retrieve and iterate through all children
        #
        # *** NOTES ***
        # The two nodes that typically have children are category nodes and
        # enumeration nodes. Throughout the examples, the children of category nodes
        # are referred to as features while the children of enumeration nodes are
        # referred to as entries. Keep in mind that enumeration nodes can be cast as
        # category nodes, but category nodes cannot be cast as enumerations.
        for node_feature in node_category.GetFeatures():

            # Ensure node is readable
            if not PySpin.IsReadable(node_feature):
                continue
            
            # Category nodes must be dealt with separately in order to retrieve subnodes recursively.
            if node_feature.GetPrincipalInterfaceType() == PySpin.intfICategory:
                result &= print_category_node_and_all_features(node_feature, level + 1)

            # Cast all non-category nodes as value nodes
            #
            # *** NOTES ***
            # If dealing with a variety of node types and their values, it may be
            # simpler to cast them as value nodes rather than as their individual types.
            # However, with this increased ease-of-use, functionality is sacrificed.
            elif CHOSEN_READ == ReadType.VALUE:
                result &= print_value_node(node_feature, level + 1)

            # Cast all non-category nodes as actual types
            elif CHOSEN_READ == ReadType.INDIVIDUAL:
                if node_feature.GetPrincipalInterfaceType() == PySpin.intfIString:
                    result &= print_string_node(node_feature, level + 1)
                elif node_feature.GetPrincipalInterfaceType() == PySpin.intfIInteger:
                    result &= print_integer_node(node_feature, level + 1)
                elif node_feature.GetPrincipalInterfaceType() == PySpin.intfIFloat:
                    result &= print_float_node(node_feature, level + 1)
                elif node_feature.GetPrincipalInterfaceType() == PySpin.intfIBoolean:
                    result &= print_boolean_node(node_feature, level + 1)
                elif node_feature.GetPrincipalInterfaceType() == PySpin.intfICommand:
                    result &= print_command_node(node_feature, level + 1)
                elif node_feature.GetPrincipalInterfaceType() == PySpin.intfIEnumeration:
                    result &= print_enumeration_node_and_current_entry(node_feature, level + 1)

        print('')

    except PySpin.SpinnakerException as ex:
        print('Error: %s' % ex)
        return False

    return result
# _________________________________________________