from http import HTTPStatus
from typing import Any
from urllib.parse import quote

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.arm_side import ArmSide
from ...models.error_envelope import ErrorEnvelope
from ...models.gripper_clear_fault_response_200 import GripperClearFaultResponse200
from ...types import Response


def _get_kwargs(
    side: ArmSide,
) -> dict[str, Any]:

    _kwargs: dict[str, Any] = {
        "method": "post",
        "url": "/v1/gripper/{side}/clear_fault".format(
            side=quote(str(side), safe=""),
        ),
    }

    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ErrorEnvelope | GripperClearFaultResponse200 | None:
    if response.status_code == 200:
        response_200 = GripperClearFaultResponse200.from_dict(response.json())

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
) -> Response[ErrorEnvelope | GripperClearFaultResponse200]:
    return Response(
        status_code=HTTPStatus(response.status_code),
        content=response.content,
        headers=response.headers,
        parsed=_parse_response(client=client, response=response),
    )


def sync_detailed(
    side: ArmSide,
    *,
    client: AuthenticatedClient | Client,
) -> Response[ErrorEnvelope | GripperClearFaultResponse200]:
    """Clear one gripper motor's fault latch

     The grippers' counterpart of `/v1/arm/{side}/clear_errors`. A gripper motor that has latched a fault
    swallows position commands until the latch is cleared, so every stroke after it is accepted and then
    ends immediately as `kind: fault` at the anchor read — before this existed the only recovery was a
    power cycle of the gripper. Sends the motor's clear-error frame, re-reads the status and answers
    with it: `cleared` is the success flag and `status` the decoded status nibble. It commands no motion
    and does not re-enable the motor; the next stroke arms it. Not gated by the gripper soft-kill latch,
    for the same reason `clear_errors` is not. A `coil_over_temperature` latch does not clear in
    software at all: it answers `cleared: false` until the coil has cooled. Strokes also attempt one
    clear of their own when they find the motor faulted, so this is for the case where that did not
    take.

    Args:
        side (ArmSide): Selects one of the two physical arms.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ErrorEnvelope | GripperClearFaultResponse200]
    """

    kwargs = _get_kwargs(
        side=side,
    )

    response = client.get_httpx_client().request(
        **kwargs,
    )

    return _build_response(client=client, response=response)


def sync(
    side: ArmSide,
    *,
    client: AuthenticatedClient | Client,
) -> ErrorEnvelope | GripperClearFaultResponse200 | None:
    """Clear one gripper motor's fault latch

     The grippers' counterpart of `/v1/arm/{side}/clear_errors`. A gripper motor that has latched a fault
    swallows position commands until the latch is cleared, so every stroke after it is accepted and then
    ends immediately as `kind: fault` at the anchor read — before this existed the only recovery was a
    power cycle of the gripper. Sends the motor's clear-error frame, re-reads the status and answers
    with it: `cleared` is the success flag and `status` the decoded status nibble. It commands no motion
    and does not re-enable the motor; the next stroke arms it. Not gated by the gripper soft-kill latch,
    for the same reason `clear_errors` is not. A `coil_over_temperature` latch does not clear in
    software at all: it answers `cleared: false` until the coil has cooled. Strokes also attempt one
    clear of their own when they find the motor faulted, so this is for the case where that did not
    take.

    Args:
        side (ArmSide): Selects one of the two physical arms.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ErrorEnvelope | GripperClearFaultResponse200
    """

    return sync_detailed(
        side=side,
        client=client,
    ).parsed


async def asyncio_detailed(
    side: ArmSide,
    *,
    client: AuthenticatedClient | Client,
) -> Response[ErrorEnvelope | GripperClearFaultResponse200]:
    """Clear one gripper motor's fault latch

     The grippers' counterpart of `/v1/arm/{side}/clear_errors`. A gripper motor that has latched a fault
    swallows position commands until the latch is cleared, so every stroke after it is accepted and then
    ends immediately as `kind: fault` at the anchor read — before this existed the only recovery was a
    power cycle of the gripper. Sends the motor's clear-error frame, re-reads the status and answers
    with it: `cleared` is the success flag and `status` the decoded status nibble. It commands no motion
    and does not re-enable the motor; the next stroke arms it. Not gated by the gripper soft-kill latch,
    for the same reason `clear_errors` is not. A `coil_over_temperature` latch does not clear in
    software at all: it answers `cleared: false` until the coil has cooled. Strokes also attempt one
    clear of their own when they find the motor faulted, so this is for the case where that did not
    take.

    Args:
        side (ArmSide): Selects one of the two physical arms.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ErrorEnvelope | GripperClearFaultResponse200]
    """

    kwargs = _get_kwargs(
        side=side,
    )

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    side: ArmSide,
    *,
    client: AuthenticatedClient | Client,
) -> ErrorEnvelope | GripperClearFaultResponse200 | None:
    """Clear one gripper motor's fault latch

     The grippers' counterpart of `/v1/arm/{side}/clear_errors`. A gripper motor that has latched a fault
    swallows position commands until the latch is cleared, so every stroke after it is accepted and then
    ends immediately as `kind: fault` at the anchor read — before this existed the only recovery was a
    power cycle of the gripper. Sends the motor's clear-error frame, re-reads the status and answers
    with it: `cleared` is the success flag and `status` the decoded status nibble. It commands no motion
    and does not re-enable the motor; the next stroke arms it. Not gated by the gripper soft-kill latch,
    for the same reason `clear_errors` is not. A `coil_over_temperature` latch does not clear in
    software at all: it answers `cleared: false` until the coil has cooled. Strokes also attempt one
    clear of their own when they find the motor faulted, so this is for the case where that did not
    take.

    Args:
        side (ArmSide): Selects one of the two physical arms.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ErrorEnvelope | GripperClearFaultResponse200
    """

    return (
        await asyncio_detailed(
            side=side,
            client=client,
        )
    ).parsed
