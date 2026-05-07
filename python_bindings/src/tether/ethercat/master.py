"""
Tether EtherCAT Master Module

EtherCAT master implementation for network scanning, slave configuration,
and real-time communication.

Classes:
    Master: Main EtherCAT master class
    NetworkScanner: Scan and enumerate slaves
    ProcessDataHandler: PDO exchange
    MailboxHandler: SDO, FoE, EoE, etc.

Example:
    >>> from tether.ethercat import master
    >>>
    >>> # Initialize master
    >>> m = master.Master()
    >>> m.init("eth0")
    >>>
    >>> # Scan network
    >>> slaves = m.scan_network()
    >>> for slave in slaves:
    ...     print(f"Slave {slave.index}: {slave.name}")
    >>>
    >>> # Configure and start
    >>> m.configure_dc()
    >>> m.set_state(master.SlaveState.OP)
"""

# Import the native extension - handle both development and installed paths
try:
    try:
        from tether._ethercat_master import *
    except ImportError:
        from _ethercat_master import *
except ImportError as e:
    raise ImportError(
        "Failed to import _ethercat_master native module. "
        "Ensure the package is properly installed or the native module is in the path."
    ) from e

__all__ = [
    "placeholder",  # From stub
]