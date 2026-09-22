from http import HTTPStatus
from typing import Any
from urllib.parse import quote

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.arm_side import ArmSide
from ...models.error_envelope import ErrorEnvelope
from ...models.grip_request import GripRequest
from ...models.gripper_close_response_200 import GripperCloseResponse200
from ...types import UNSET, Response, Unset


def _get_kwargs(
    side: ArmSide,
    *,
    body: GripRequest | Unset = UNSET,
) -> dict[str, Any]:
    headers: dict[str, Any] = {}

    _kwargs: dict[str, Any] = {
        "method": "post",
        "url": "/v1/gripper/{side}/close".format(
            side=quote(str(side), safe=""),
        ),
    }

    if not isinstance(body, Unset):
        _kwargs["json"] = body.to_dict()

    headers["Content-Type"] = "application/json"

    _kwargs["headers"] = headers
    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ErrorEnvelope | GripperCloseResponse200 | None:
    if response.status_code == 200:
        response_200 = GripperCloseResponse200.from_dict(response.json())

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
) -> Response[ErrorEnvelope | GripperCloseResponse200]:
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
    body: GripRequest | Unset = UNSET,
) -> Response[ErrorEnvelope | GripperCloseResponse200]:
    """Close one gripper

     Blocks for the stroke and returns nothing; read `/v1/gripper/{side}/state` for the resulting report.
    With no body this is the backend's default grip; with a body naming a grip it is a full-closedness
    target using that grip.

    Args:
        side (ArmSide): Selects one of the two physical arms.
        body (GripRequest | Unset): How hard a closing stroke squeezes and holds what it meets.

            At most one of the two fields is given: a named [`GripPreset`], or an
            explicit standing preload in motor radians (which keeps the `firm`
            preset's stop torque). Neither means the backend's configured default,
            `firm`. Carried by [`GripperTarget`] and by a bare `close`.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ErrorEnvelope | GripperCloseResponse200]
    """

    kwargs = _get_kwargs(
        side=side,
        body=body,
    )

    response = client.get_httpx_client().request(
        **kwargs,
    )

    return _build_response(client=client, response=response)


def sync(
    side: ArmSide,
    *,
    client: AuthenticatedClient | Client,
    body: GripRequest | Unset = UNSET,
) -> ErrorEnvelope | GripperCloseResponse200 | None:
    """Close one gripper

     Blocks for the stroke and returns nothing; read `/v1/gripper/{side}/state` for the resulting report.
    With no body this is the backend's default grip; with a body naming a grip it is a full-closedness
    target using that grip.

    Args:
        side (ArmSide): Selects one of the two physical arms.
        body (GripRequest | Unset): How hard a closing stroke squeezes and holds what it meets.

            At most one of the two fields is given: a named [`GripPreset`], or an
            explicit standing preload in motor radians (which keeps the `firm`
            preset's stop torque). Neither means the backend's configured default,
            `firm`. Carried by [`GripperTarget`] and by a bare `close`.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ErrorEnvelope | GripperCloseResponse200
    """

    return sync_detailed(
        side=side,
        client=client,
        body=body,
    ).parsed


async def asyncio_detailed(
    side: ArmSide,
    *,
    client: AuthenticatedClient | Client,
    body: GripRequest | Unset = UNSET,
) -> Response[ErrorEnvelope | GripperCloseResponse200]:
    """Close one gripper

     Blocks for the stroke and returns nothing; read `/v1/gripper/{side}/state` for the resulting report.
    With no body this is the backend's default grip; with a body naming a grip it is a full-closedness
    target using that grip.

    Args:
        side (ArmSide): Selects one of the two physical arms.
        body (GripRequest | Unset): How hard a closing stroke squeezes and holds what it meets.

            At most one of the two fields is given: a named [`GripPreset`], or an
            explicit standing preload in motor radians (which keeps the `firm`
            preset's stop torque). Neither means the backend's configured default,
            `firm`. Carried by [`GripperTarget`] and by a bare `close`.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ErrorEnvelope | GripperCloseResponse200]
    """

    kwargs = _get_kwargs(
        side=side,
        body=body,
    )

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    side: ArmSide,
    *,
    client: AuthenticatedClient | Client,
    body: GripRequest | Unset = UNSET,
) -> ErrorEnvelope | GripperCloseResponse200 | None:
    """Close one gripper

     Blocks for the stroke and returns nothing; read `/v1/gripper/{side}/state` for the resulting report.
    With no body this is the backend's default grip; with a body naming a grip it is a full-closedness
    target using that grip.

    Args:
        side (ArmSide): Selects one of the two physical arms.
        body (GripRequest | Unset): How hard a closing stroke squeezes and holds what it meets.

            At most one of the two fields is given: a named [`GripPreset`], or an
            explicit standing preload in motor radians (which keeps the `firm`
            preset's stop torque). Neither means the backend's configured default,
            `firm`. Carried by [`GripperTarget`] and by a bare `close`.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ErrorEnvelope | GripperCloseResponse200
    """

    return (
        await asyncio_detailed(
            side=side,
            client=client,
            body=body,
        )
    ).parsed
