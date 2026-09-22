from http import HTTPStatus
from typing import Any

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.chassis_control_get_response_200 import ChassisControlGetResponse200
from ...models.error_envelope import ErrorEnvelope
from ...types import Response


def _get_kwargs() -> dict[str, Any]:

    _kwargs: dict[str, Any] = {
        "method": "get",
        "url": "/v1/chassis/control",
    }

    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ChassisControlGetResponse200 | ErrorEnvelope | None:
    if response.status_code == 200:
        response_200 = ChassisControlGetResponse200.from_dict(response.json())

        return response_200

    if response.status_code == 400:
        response_400 = ErrorEnvelope.from_dict(response.json())

        return response_400

    if response.status_code == 404:
        response_404 = ErrorEnvelope.from_dict(response.json())

        return response_404

    if response.status_code == 409:
        response_409 = ErrorEnvelope.from_dict(response.json())

        return response_409

    if response.status_code == 502:
        response_502 = ErrorEnvelope.from_dict(response.json())

        return response_502

    if response.status_code == 504:
        response_504 = ErrorEnvelope.from_dict(response.json())

        return response_504

    if client.raise_on_unexpected_status:
        raise errors.UnexpectedStatus(response.status_code, response.content)
    else:
        return None


def _build_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> Response[ChassisControlGetResponse200 | ErrorEnvelope]:
    return Response(
        status_code=HTTPStatus(response.status_code),
        content=response.content,
        headers=response.headers,
        parsed=_parse_response(client=client, response=response),
    )


def sync_detailed(
    *,
    client: AuthenticatedClient | Client,
) -> Response[ChassisControlGetResponse200 | ErrorEnvelope]:
    """Read which controller may drive the mobile base

     The mobile base has exactly ONE control mode at a time and enforces it by IGNORING commands: outside
    its remote-control mode (`workMode` 4) it answers a jog or a velocity command with HTTP 200 and does
    not move. This is that mode, decoded, with the control preconditions for sending motion commands, so
    an operator interface can poll one object at 1 Hz instead of inferring the answer from a digit. The
    vendor's five modes are its own: `idle` (0, navigation closed), `navigation` (1), `auto_charging`
    (2), `mapping` (3), `remote_control` (4). `navigation` is the vendor's `autoStatus` decoded, and is
    `off` in every mode but 1, because the base does NOT clear that field when it leaves navigation -- a
    stale `moving` read in remote-control mode is the trap this closes; the untouched value is in
    `auto_status_raw`. `can_hand_drive` is the one-field answer to "do control preconditions permit
    sending a jog": true only in remote control, not reported in standby, with the emergency switch
    released and the motors not released by this daemon, and `hand_drive_blockers` names each failing
    precondition with the sentence that clears it. Unknown motor and standby states do not block. This
    is permission to send before the separate software stop gate, not proof of physical movement: motor
    command memory can be stale and the base's bumper interlock can stop motion without blocking
    transmission. `motors` is NOT a reading from the base: the vendor's `lockCtrl` endpoint writes the
    motor lock and nothing reads it back, so it is this daemon's record of its own last successful write
    and `motors_source` is `unknown` until there has been one. `transitions` lists every control verb
    with whether it is legal from this state and why not -- it describes the MOBILE BASE's mode machine
    only, and this daemon's chassis soft-kill latch can still refuse an allowed verb with the
    `kill_latched` kind. The same object is the `control` field of `GET /v1/chassis/state`. Not gated by
    the chassis soft-kill latch.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ChassisControlGetResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs()

    response = client.get_httpx_client().request(
        **kwargs,
    )

    return _build_response(client=client, response=response)


def sync(
    *,
    client: AuthenticatedClient | Client,
) -> ChassisControlGetResponse200 | ErrorEnvelope | None:
    """Read which controller may drive the mobile base

     The mobile base has exactly ONE control mode at a time and enforces it by IGNORING commands: outside
    its remote-control mode (`workMode` 4) it answers a jog or a velocity command with HTTP 200 and does
    not move. This is that mode, decoded, with the control preconditions for sending motion commands, so
    an operator interface can poll one object at 1 Hz instead of inferring the answer from a digit. The
    vendor's five modes are its own: `idle` (0, navigation closed), `navigation` (1), `auto_charging`
    (2), `mapping` (3), `remote_control` (4). `navigation` is the vendor's `autoStatus` decoded, and is
    `off` in every mode but 1, because the base does NOT clear that field when it leaves navigation -- a
    stale `moving` read in remote-control mode is the trap this closes; the untouched value is in
    `auto_status_raw`. `can_hand_drive` is the one-field answer to "do control preconditions permit
    sending a jog": true only in remote control, not reported in standby, with the emergency switch
    released and the motors not released by this daemon, and `hand_drive_blockers` names each failing
    precondition with the sentence that clears it. Unknown motor and standby states do not block. This
    is permission to send before the separate software stop gate, not proof of physical movement: motor
    command memory can be stale and the base's bumper interlock can stop motion without blocking
    transmission. `motors` is NOT a reading from the base: the vendor's `lockCtrl` endpoint writes the
    motor lock and nothing reads it back, so it is this daemon's record of its own last successful write
    and `motors_source` is `unknown` until there has been one. `transitions` lists every control verb
    with whether it is legal from this state and why not -- it describes the MOBILE BASE's mode machine
    only, and this daemon's chassis soft-kill latch can still refuse an allowed verb with the
    `kill_latched` kind. The same object is the `control` field of `GET /v1/chassis/state`. Not gated by
    the chassis soft-kill latch.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ChassisControlGetResponse200 | ErrorEnvelope
    """

    return sync_detailed(
        client=client,
    ).parsed


async def asyncio_detailed(
    *,
    client: AuthenticatedClient | Client,
) -> Response[ChassisControlGetResponse200 | ErrorEnvelope]:
    """Read which controller may drive the mobile base

     The mobile base has exactly ONE control mode at a time and enforces it by IGNORING commands: outside
    its remote-control mode (`workMode` 4) it answers a jog or a velocity command with HTTP 200 and does
    not move. This is that mode, decoded, with the control preconditions for sending motion commands, so
    an operator interface can poll one object at 1 Hz instead of inferring the answer from a digit. The
    vendor's five modes are its own: `idle` (0, navigation closed), `navigation` (1), `auto_charging`
    (2), `mapping` (3), `remote_control` (4). `navigation` is the vendor's `autoStatus` decoded, and is
    `off` in every mode but 1, because the base does NOT clear that field when it leaves navigation -- a
    stale `moving` read in remote-control mode is the trap this closes; the untouched value is in
    `auto_status_raw`. `can_hand_drive` is the one-field answer to "do control preconditions permit
    sending a jog": true only in remote control, not reported in standby, with the emergency switch
    released and the motors not released by this daemon, and `hand_drive_blockers` names each failing
    precondition with the sentence that clears it. Unknown motor and standby states do not block. This
    is permission to send before the separate software stop gate, not proof of physical movement: motor
    command memory can be stale and the base's bumper interlock can stop motion without blocking
    transmission. `motors` is NOT a reading from the base: the vendor's `lockCtrl` endpoint writes the
    motor lock and nothing reads it back, so it is this daemon's record of its own last successful write
    and `motors_source` is `unknown` until there has been one. `transitions` lists every control verb
    with whether it is legal from this state and why not -- it describes the MOBILE BASE's mode machine
    only, and this daemon's chassis soft-kill latch can still refuse an allowed verb with the
    `kill_latched` kind. The same object is the `control` field of `GET /v1/chassis/state`. Not gated by
    the chassis soft-kill latch.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ChassisControlGetResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs()

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    *,
    client: AuthenticatedClient | Client,
) -> ChassisControlGetResponse200 | ErrorEnvelope | None:
    """Read which controller may drive the mobile base

     The mobile base has exactly ONE control mode at a time and enforces it by IGNORING commands: outside
    its remote-control mode (`workMode` 4) it answers a jog or a velocity command with HTTP 200 and does
    not move. This is that mode, decoded, with the control preconditions for sending motion commands, so
    an operator interface can poll one object at 1 Hz instead of inferring the answer from a digit. The
    vendor's five modes are its own: `idle` (0, navigation closed), `navigation` (1), `auto_charging`
    (2), `mapping` (3), `remote_control` (4). `navigation` is the vendor's `autoStatus` decoded, and is
    `off` in every mode but 1, because the base does NOT clear that field when it leaves navigation -- a
    stale `moving` read in remote-control mode is the trap this closes; the untouched value is in
    `auto_status_raw`. `can_hand_drive` is the one-field answer to "do control preconditions permit
    sending a jog": true only in remote control, not reported in standby, with the emergency switch
    released and the motors not released by this daemon, and `hand_drive_blockers` names each failing
    precondition with the sentence that clears it. Unknown motor and standby states do not block. This
    is permission to send before the separate software stop gate, not proof of physical movement: motor
    command memory can be stale and the base's bumper interlock can stop motion without blocking
    transmission. `motors` is NOT a reading from the base: the vendor's `lockCtrl` endpoint writes the
    motor lock and nothing reads it back, so it is this daemon's record of its own last successful write
    and `motors_source` is `unknown` until there has been one. `transitions` lists every control verb
    with whether it is legal from this state and why not -- it describes the MOBILE BASE's mode machine
    only, and this daemon's chassis soft-kill latch can still refuse an allowed verb with the
    `kill_latched` kind. The same object is the `control` field of `GET /v1/chassis/state`. Not gated by
    the chassis soft-kill latch.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ChassisControlGetResponse200 | ErrorEnvelope
    """

    return (
        await asyncio_detailed(
            client=client,
        )
    ).parsed
