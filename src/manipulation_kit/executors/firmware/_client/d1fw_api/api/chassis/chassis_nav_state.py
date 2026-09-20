from http import HTTPStatus
from typing import Any
from urllib.parse import quote

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.chassis_nav_state_response_200 import ChassisNavStateResponse200
from ...models.error_envelope import ErrorEnvelope
from ...types import Response


def _get_kwargs(
    goal_id: int,
) -> dict[str, Any]:

    _kwargs: dict[str, Any] = {
        "method": "get",
        "url": "/v1/chassis/navigate/{goal_id}".format(
            goal_id=quote(str(goal_id), safe=""),
        ),
    }

    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ChassisNavStateResponse200 | ErrorEnvelope | None:
    if response.status_code == 200:
        response_200 = ChassisNavStateResponse200.from_dict(response.json())

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
) -> Response[ChassisNavStateResponse200 | ErrorEnvelope]:
    return Response(
        status_code=HTTPStatus(response.status_code),
        content=response.content,
        headers=response.headers,
        parsed=_parse_response(client=client, response=response),
    )


def sync_detailed(
    goal_id: int,
    *,
    client: AuthenticatedClient | Client,
) -> Response[ChassisNavStateResponse200 | ErrorEnvelope]:
    """Poll one navigation goal's progress

    Args:
        goal_id (int):

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ChassisNavStateResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs(
        goal_id=goal_id,
    )

    response = client.get_httpx_client().request(
        **kwargs,
    )

    return _build_response(client=client, response=response)


def sync(
    goal_id: int,
    *,
    client: AuthenticatedClient | Client,
) -> ChassisNavStateResponse200 | ErrorEnvelope | None:
    """Poll one navigation goal's progress

    Args:
        goal_id (int):

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ChassisNavStateResponse200 | ErrorEnvelope
    """

    return sync_detailed(
        goal_id=goal_id,
        client=client,
    ).parsed


async def asyncio_detailed(
    goal_id: int,
    *,
    client: AuthenticatedClient | Client,
) -> Response[ChassisNavStateResponse200 | ErrorEnvelope]:
    """Poll one navigation goal's progress

    Args:
        goal_id (int):

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ChassisNavStateResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs(
        goal_id=goal_id,
    )

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    goal_id: int,
    *,
    client: AuthenticatedClient | Client,
) -> ChassisNavStateResponse200 | ErrorEnvelope | None:
    """Poll one navigation goal's progress

    Args:
        goal_id (int):

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ChassisNavStateResponse200 | ErrorEnvelope
    """

    return (
        await asyncio_detailed(
            goal_id=goal_id,
            client=client,
        )
    ).parsed
