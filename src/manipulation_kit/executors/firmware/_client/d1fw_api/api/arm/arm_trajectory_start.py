from http import HTTPStatus
from typing import Any

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.arm_trajectory_start_response_200 import ArmTrajectoryStartResponse200
from ...models.error_envelope import ErrorEnvelope
from ...models.trajectory_request import TrajectoryRequest
from ...types import Response


def _get_kwargs(
    *,
    body: TrajectoryRequest,
) -> dict[str, Any]:
    headers: dict[str, Any] = {}

    _kwargs: dict[str, Any] = {
        "method": "post",
        "url": "/v1/arm/trajectory/start",
    }

    _kwargs["json"] = body.to_dict()

    headers["Content-Type"] = "application/json"

    _kwargs["headers"] = headers
    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ArmTrajectoryStartResponse200 | ErrorEnvelope | None:
    if response.status_code == 200:
        response_200 = ArmTrajectoryStartResponse200.from_dict(response.json())

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
) -> Response[ArmTrajectoryStartResponse200 | ErrorEnvelope]:
    return Response(
        status_code=HTTPStatus(response.status_code),
        content=response.content,
        headers=response.headers,
        parsed=_parse_response(client=client, response=response),
    )


def sync_detailed(
    *,
    client: AuthenticatedClient | Client,
    body: TrajectoryRequest,
) -> Response[ArmTrajectoryStartResponse200 | ErrorEnvelope]:
    """Start a guarded dual-arm trajectory

     Returns as soon as the job is accepted; poll `/v1/arm/trajectory/{id}/status`. Only one job exists
    at a time and only the most recent job's status is retained. The arms must already be in a clean
    position or torque hold within 3 degrees of the first waypoint. Not available over WebSocket.

    Args:
        body (TrajectoryRequest): A bounded trajectory upload. Mode and tool selection remain
            explicit verbs.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ArmTrajectoryStartResponse200 | ErrorEnvelope]
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
    body: TrajectoryRequest,
) -> ArmTrajectoryStartResponse200 | ErrorEnvelope | None:
    """Start a guarded dual-arm trajectory

     Returns as soon as the job is accepted; poll `/v1/arm/trajectory/{id}/status`. Only one job exists
    at a time and only the most recent job's status is retained. The arms must already be in a clean
    position or torque hold within 3 degrees of the first waypoint. Not available over WebSocket.

    Args:
        body (TrajectoryRequest): A bounded trajectory upload. Mode and tool selection remain
            explicit verbs.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ArmTrajectoryStartResponse200 | ErrorEnvelope
    """

    return sync_detailed(
        client=client,
        body=body,
    ).parsed


async def asyncio_detailed(
    *,
    client: AuthenticatedClient | Client,
    body: TrajectoryRequest,
) -> Response[ArmTrajectoryStartResponse200 | ErrorEnvelope]:
    """Start a guarded dual-arm trajectory

     Returns as soon as the job is accepted; poll `/v1/arm/trajectory/{id}/status`. Only one job exists
    at a time and only the most recent job's status is retained. The arms must already be in a clean
    position or torque hold within 3 degrees of the first waypoint. Not available over WebSocket.

    Args:
        body (TrajectoryRequest): A bounded trajectory upload. Mode and tool selection remain
            explicit verbs.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ArmTrajectoryStartResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs(
        body=body,
    )

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    *,
    client: AuthenticatedClient | Client,
    body: TrajectoryRequest,
) -> ArmTrajectoryStartResponse200 | ErrorEnvelope | None:
    """Start a guarded dual-arm trajectory

     Returns as soon as the job is accepted; poll `/v1/arm/trajectory/{id}/status`. Only one job exists
    at a time and only the most recent job's status is retained. The arms must already be in a clean
    position or torque hold within 3 degrees of the first waypoint. Not available over WebSocket.

    Args:
        body (TrajectoryRequest): A bounded trajectory upload. Mode and tool selection remain
            explicit verbs.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ArmTrajectoryStartResponse200 | ErrorEnvelope
    """

    return (
        await asyncio_detailed(
            client=client,
            body=body,
        )
    ).parsed
