from http import HTTPStatus
from typing import Any

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.chassis_ros_state_response_200 import ChassisRosStateResponse200
from ...models.error_envelope import ErrorEnvelope
from ...types import Response


def _get_kwargs() -> dict[str, Any]:

    _kwargs: dict[str, Any] = {
        "method": "get",
        "url": "/v1/chassis/ros/state",
    }

    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ChassisRosStateResponse200 | ErrorEnvelope | None:
    if response.status_code == 200:
        response_200 = ChassisRosStateResponse200.from_dict(response.json())

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
) -> Response[ChassisRosStateResponse200 | ErrorEnvelope]:
    return Response(
        status_code=HTTPStatus(response.status_code),
        content=response.content,
        headers=response.headers,
        parsed=_parse_response(client=client, response=response),
    )


def sync_detailed(
    *,
    client: AuthenticatedClient | Client,
) -> Response[ChassisRosStateResponse200 | ErrorEnvelope]:
    """Vendor ROS state

     One snapshot per subscribed manufacturer ROS topic, with receipt times, freshness and per-topic
    errors. The subscription set is exactly the control-state topics the base's rosbridge allow-list
    serves; the audited topics it does not serve are reported by `chassis_ros_capabilities` as
    unsupported rather than appearing here as permanently silent. Never received does not prove absence.
    Separate from the D1 ROS2 frontend.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ChassisRosStateResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs()

    response = client.get_httpx_client().request(
        **kwargs,
    )

    return _build_response(client=client, response=response)


def sync(
    *,
    client: AuthenticatedClient | Client,
) -> ChassisRosStateResponse200 | ErrorEnvelope | None:
    """Vendor ROS state

     One snapshot per subscribed manufacturer ROS topic, with receipt times, freshness and per-topic
    errors. The subscription set is exactly the control-state topics the base's rosbridge allow-list
    serves; the audited topics it does not serve are reported by `chassis_ros_capabilities` as
    unsupported rather than appearing here as permanently silent. Never received does not prove absence.
    Separate from the D1 ROS2 frontend.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ChassisRosStateResponse200 | ErrorEnvelope
    """

    return sync_detailed(
        client=client,
    ).parsed


async def asyncio_detailed(
    *,
    client: AuthenticatedClient | Client,
) -> Response[ChassisRosStateResponse200 | ErrorEnvelope]:
    """Vendor ROS state

     One snapshot per subscribed manufacturer ROS topic, with receipt times, freshness and per-topic
    errors. The subscription set is exactly the control-state topics the base's rosbridge allow-list
    serves; the audited topics it does not serve are reported by `chassis_ros_capabilities` as
    unsupported rather than appearing here as permanently silent. Never received does not prove absence.
    Separate from the D1 ROS2 frontend.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ChassisRosStateResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs()

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    *,
    client: AuthenticatedClient | Client,
) -> ChassisRosStateResponse200 | ErrorEnvelope | None:
    """Vendor ROS state

     One snapshot per subscribed manufacturer ROS topic, with receipt times, freshness and per-topic
    errors. The subscription set is exactly the control-state topics the base's rosbridge allow-list
    serves; the audited topics it does not serve are reported by `chassis_ros_capabilities` as
    unsupported rather than appearing here as permanently silent. Never received does not prove absence.
    Separate from the D1 ROS2 frontend.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ChassisRosStateResponse200 | ErrorEnvelope
    """

    return (
        await asyncio_detailed(
            client=client,
        )
    ).parsed
