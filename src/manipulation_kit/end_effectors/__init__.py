"""manipulation_kit.end_effectors — end effectors served to d1-firmwared by a
process of their own, over the ``end_effector.shm/1`` shared-memory contract.

* :mod:`~manipulation_kit.end_effectors.shm_contract` — the contract's
  messages as ctypes structures, checked against the vendored layout. Pure
  computation.
* :mod:`~manipulation_kit.end_effectors.shm_provider` — the provider runtime
  over the iceoryx2 0.9.3 Python bindings (the optional ``[shm]`` extra),
  two simulated end effectors, and the ``mkit-ee-provider`` command.

Nothing here is imported by the rest of the package, and importing this
package opens nothing: the bindings are imported only when a provider is
served. See ``docs/end-effector-providers.md``.
"""
