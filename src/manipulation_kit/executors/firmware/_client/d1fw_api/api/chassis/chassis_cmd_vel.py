from http import HTTPStatus
from typing import Any

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.chassis_cmd_vel_response_200 import ChassisCmdVelResponse200
from ...models.chassis_velocity import ChassisVelocity
from ...models.error_envelope import ErrorEnvelope
from ...types import Response


def _get_kwargs(
    *,
    body: ChassisVelocity,
) -> dict[str, Any]:
    headers: dict[str, Any] = {}

    _kwargs: dict[str, Any] = {
        "method": "post",
        "url": "/v1/chassis/cmd_vel",
    }

    _kwargs["json"] = body.to_dict()

    headers["Content-Type"] = "application/json"

    _kwargs["headers"] = headers
    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ChassisCmdVelResponse200 | ErrorEnvelope | None:
    if response.status_code == 200:
        response_200 = ChassisCmdVelResponse200.from_dict(response.json())

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
) -> Response[ChassisCmdVelResponse200 | ErrorEnvelope]:
    return Response(
        status_code=HTTPStatus(response.status_code),
        content=response.content,
        headers=response.headers,
        parsed=_parse_response(client=client, response=response),
    )


def sync_detailed(
    *,
    client: AuthenticatedClient | Client,
    body: ChassisVelocity,
) -> Response[ChassisCmdVelResponse200 | ErrorEnvelope]:
    """Command a chassis body velocity

     One command, not a stream: the chassis holds the last commanded velocity, so a caller driving
    continuously must keep sending and must send a zero velocity to stop.

    Args:
        body (ChassisVelocity): A chassis velocity command.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ChassisCmdVelResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs(
        body=body,
    )

    response = client.get_httpx_client().request(
        **kwargs,
    )

    return _build_response(client=client, response=response)


def sync(
    *,
    client: AuthenticatedClient | Client,
    body: ChassisVelocity,
) -> ChassisCmdVelResponse200 | ErrorEnvelope | None:
    """Command a chassis body velocity

     One command, not a stream: the chassis holds the last commanded velocity, so a caller driving
    continuously must keep sending and must send a zero velocity to stop.

    Args:
        body (ChassisVelocity): A chassis velocity command.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ChassisCmdVelResponse200 | ErrorEnvelope
    """

    return sync_detailed(
        client=client,
        body=body,
    ).parsed


async def asyncio_detailed(
    *,
    client: AuthenticatedClient | Client,
    body: ChassisVelocity,
) -> Response[ChassisCmdVelResponse200 | ErrorEnvelope]:
    """Command a chassis body velocity

     One command, not a stream: the chassis holds the last commanded velocity, so a caller driving
    continuously must keep sending and must send a zero velocity to stop.

    Args:
        body (ChassisVelocity): A chassis velocity command.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ChassisCmdVelResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs(
        body=body,
    )

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    *,
    client: AuthenticatedClient | Client,
    body: ChassisVelocity,
) -> ChassisCmdVelResponse200 | ErrorEnvelope | None:
    """Command a chassis body velocity

     One command, not a stream: the chassis holds the last commanded velocity, so a caller driving
    continuously must keep sending and must send a zero velocity to stop.

    Args:
        body (ChassisVelocity): A chassis velocity command.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ChassisCmdVelResponse200 | ErrorEnvelope
    """

    return (
        await asyncio_detailed(
            client=client,
            body=body,
        )
    ).parsed
